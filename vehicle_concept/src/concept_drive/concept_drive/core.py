"""Accept one synchronized perception result and advance the frozen V6.26 driver once.

All geometry here uses metres forward/right. Sensor gyro and output yaw in this
module are clockwise-positive; the ROS adapter performs the two sign conversions.
"""
from pathlib import Path

import numpy as np

from concept_candidates.backtrack_forward617 import BacktrackSteer
from concept_candidates.curve_progress import build_reference
from concept_candidates.edge_recovery import fit_edge
from concept_candidates.planner625 import MarkedPlanner
from concept_candidates.ranked_builder617 import RankedBuilder
from concept_candidates.rear_reference617 import RecoveryReference
from concept_feasibility.response61515 import physical_applied_command
from concept_obstacles.obstacles617 import Occupancy617, instances
from concept_ranker import trajectory624
from concept_road.field_road import FieldRoad
from concept_road.limits6251 import JunctionLimits, observed_direction
from concept_road.map_semantics618 import SemanticMap
from concept_road.rear_road import RearRoad
from concept_road.runtime_map import RuntimeMap, lane_sections
from concept_road.vision import CX, CY, FOCAL, HEIGHT


class Drive:
    def __init__(self, pace=3.4):
        self.observer = RearRoad()
        self.tracker = Occupancy617()
        self.mapper = RuntimeMap()
        self.semantics = SemanticMap((FOCAL, CX, CY, HEIGHT))
        self.limits = JunctionLimits()
        self.field_road = FieldRoad()
        self.planner = MarkedPlanner(dict(pace=pace, uncertainty=1., yaw_limit=1.55))
        self.builder = RankedBuilder(BacktrackSteer(self.planner))
        weights = Path(trajectory624.__file__).with_name('model-h12.json')
        self.ranker = trajectory624.TrajectorySelector(weights, pace)
        self.set_pace(pace)
        self.previous_time = None
        self.previous_motion = None
        self.observed_openings = []

    def set_pace(self, pace):
        if not np.isfinite(pace) or not 0. < pace <= 6.:
            raise ValueError('target speed must be greater than zero and at most 6 m/s')
        self.planner.p['pace'] = float(pace)
        self.ranker.pace = float(pace)

    def step(self, image, raw, lane, det, sensor):
        if image.shape != (360, 640, 3) or image.dtype != np.uint8:
            raise ValueError('B1.4 requires a 640x360 BGR image')
        if raw.shape != image.shape[:2] or lane.shape != raw.shape:
            raise ValueError('road and line masks must match BGR')
        dt = .2 if self.previous_time is None else float(sensor['t'] - self.previous_time)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('capture time must strictly advance')
        speed = (sensor['leftRpm'] + sensor['rightRpm']) * np.pi * sensor['wheelRadius'] / 60
        yaw = sensor['gyro']
        motion_speed, motion_yaw = speed, yaw
        if self.previous_motion is not None:
            motion_speed = .5 * (self.previous_motion[0] + speed)
            motion_yaw = .5 * (self.previous_motion[1] + yaw)
        boxes = det['boxes'][(det['labels'] == 91) & (det['scores'] >= .5)].tolist()
        road, _, heading = self.observer.read(image, raw, lane, boxes, motion_speed, motion_yaw, dt)
        evidence = self.observer.last
        field_ground = sensor.get('field_ground_projection', False)
        measurements = []
        groups = instances(det, image.shape, image, field_ground=field_ground, measurements=measurements)
        objects = self.tracker.update(groups, motion_speed, motion_yaw, dt,
                                      minimum_body_z=float(road[0, 0]) if len(road) else None, image=image)
        parts = lane_sections(evidence, raw, (FOCAL, CX, CY, HEIGHT))
        snapshot = self.mapper.update(sensor, parts, objects, self.tracker.last)
        semantic = self.semantics.update(image, raw, lane, boxes, evidence, snapshot)
        guidance = np.asarray(evidence.get('lane_geometry_rows', [])).reshape(-1, 4)
        road, edges = self.limits.update(road, evidence['tracks'], semantic['lines'], motion_speed, motion_yaw, dt)
        heading, reference = observed_direction(road, heading, evidence['tracks'], edges)
        evidence['circuit_edges'] = edges
        launch_evidence = None
        if field_ground:
            near_rows = evidence.get('nearfield_rows', [])
            scene = self.field_road.update(image, raw, lane, road, heading, motion_speed, motion_yaw, dt,
                                           nearfield_rows=near_rows)
            road = intersect_confirmed_edges(scene.road, road, edges['edges'])
            heading, guidance, reference = scene.heading, scene.guidance, scene.reference
            # A contradictory physical limit invalidates this depth and everything
            # beyond it. Do not interpolate the planner through the missing road.
            heading = heading[:len(road)]
            if len(road):
                guidance = guidance[(guidance[:, 0] >= road[0, 0]) & (guidance[:, 0] <= road[-1, 0])]
                if reference is not None:
                    reference['_field_edge'] = fit_edge(road, scene.audit['observed_bounds'])
            else:
                guidance, reference = np.empty((0, 4)), None
            near_visibility = [row['z'] for row in near_rows
                               if row['source'] == 'historical_material_observation']
            launch_evidence = dict(stamp_s=float(sensor['t']), applied_speed_mps=physical_applied_command()[0],
                                   observed_near_z=near_visibility, visible_z=scene.audit.get('visible_z'),
                                   obstacle_clear=clear_launch_view(det, objects, measurements))
        else:
            if set(edges['edges']) == {'left', 'right'}:
                guidance = np.empty((0, 4))
            if reference is None:
                reference = build_reference(evidence)
        obs = RecoveryReference(road, objects, speed, yaw, dt, heading, guidance,
                                evidence['rear_history']['observed_limit_m'],
                                reference=reference)
        features, allowed, baseline = self.builder.prepare(obs)
        pick, ranking = baseline, None
        if self.builder.network_allowed:
            pick, ranking = self.ranker.rank(features, allowed, baseline, self.builder.planner.last['audit'], obs,
                                            **({'launch_evidence': launch_evidence} if field_ground else {}))
        control = self.builder.command(pick)
        self.previous_time = sensor['t']
        self.previous_motion = (speed, yaw)
        self.observed_openings = [item['position'] for item in semantic['junctions']]
        detail = self.builder.planner.last
        audit = detail['audit']
        table = audit.get('table', [])
        paths = audit.get('paths', [])
        scores = ranking['final_scores'] if ranking else [row[7] for row in table]
        candidates = [
            dict(index=i, allowed=bool(row[6]), selectable=bool(allowed[i]), path=paths[i],
                 speed_mps=row[0], yaw_right_radps=row[1], clearance_m=row[5], score=scores[i])
            for i, row in enumerate(table)
        ]
        return dict(selected_index=None if pick is None else int(pick),
                    whitebox_index=None if baseline is None else int(baseline), command_native=control.tolist(),
                    selected_path=paths[pick] if pick is not None and len(paths) else detail.get('path', []),
                    candidate_evaluation=audit['candidate_evaluation'],
                    candidates=candidates, feasible_count=sum(c['allowed'] for c in candidates),
                    **({'edge_candidate_index': audit.get('edge_candidate_index'),
                        'edge_candidate_schedule': (audit['schedule'][audit['edge_candidate_index']]
                                                    if audit.get('edge_candidate_index') is not None else None)}
                       if field_ground else {}),
                    reason=detail.get('reason', ''), recovery=detail.get('recovery', ''),
                    ranking_active=ranking is not None, road_rows=road.tolist(), heading=heading.tolist(),
                    **({'obstacle_measurements': measurements, 'field_road': scene.audit,
                        'launch_decision': ranking.get('launch_decision') if ranking else None} if field_ground else {}),
                    objects=objects.tolist(), tracks=self.tracker.last['tracks'], speed_estimate_mps=float(speed),
                    rejected=audit.get('rejected', {}), edges=edges['edges'], lines=semantic['lines'],
                    map=snapshot, map_semantics={k: semantic[k] for k in ('changed_markings','changed_gaps','changed_repairs')})


def intersect_confirmed_edges(camera_road, constrained, edges):
    """Keep the previously confirmed physical outer edges wherever both observers report."""
    if not len(camera_road) or not edges:
        return camera_road
    result = camera_road.copy()
    depth = result[:, 0]
    supported = np.ones(len(depth), dtype=bool)
    for side, col, sign in [('left', 1, 1.), ('right', 2, -1.)]:
        if not len(edges.get(side, [])):
            continue
        points = np.asarray(edges[side])
        inside = (depth >= points[0, 0]) & (depth <= points[-1, 0])
        if not len(constrained):
            supported &= ~inside
            continue
        measured = (depth >= constrained[0, 0]) & (depth <= constrained[-1, 0])
        supported &= ~inside | measured
        inside &= measured
        edge = sign * np.interp(depth[inside], constrained[:, 0], constrained[:, col])
        error = np.interp(depth[inside], constrained[:, 0], constrained[:, col + 2])
        original = sign * result[inside, col]
        bound = np.maximum(original + result[inside, col + 2], edge + error)
        merged = np.maximum(original, edge)
        result[inside, col] = sign * merged
        result[inside, col + 2] = bound - merged
    invalid = np.flatnonzero(~supported | (result[:, 1] >= result[:, 2]))
    return result[:invalid[0]] if len(invalid) else result


def clear_launch_view(detections, objects, measurements):
    """Keep stopping eligible for untracked or unprojectable nearby detections."""
    if len(objects) or any(m.get('within_tracking_range', True) for m in measurements):
        return False
    boxes = detections['boxes'][(detections['labels'] == 91) & (detections['scores'] >= .15)]
    for box in boxes:
        if not np.isfinite(box).all() or box[3] <= CY + 5:
            return False
        if HEIGHT * FOCAL / (box[3] - CY) <= 12.:
            return False
    return True
