"""Accept one synchronized perception result and advance the frozen V6.26 driver once.

All geometry here uses metres forward/right. Sensor gyro and output yaw in this
module are clockwise-positive; the ROS adapter performs the two sign conversions.
"""
from functools import partial
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from concept_candidates.backtrack_forward617 import BacktrackSteer
from concept_candidates.curve_progress import build_reference
from concept_candidates.edge_recovery import corroborated_previous_edge, fit_edge, preview_audit
from concept_candidates.launch_clearance import launch_clearance
from concept_candidates.tracking import controller_record, first_lya_depth, validate_mode
from concept_candidates.planner625 import MarkedPlanner
from concept_candidates.ranked_builder617 import RankedBuilder
from concept_candidates.rear_reference617 import RecoveryReference
from concept_feasibility.response61515 import physical_applied_command
from concept_obstacles.obstacles617 import Occupancy617, instances
from concept_obstacles.launch_view import launch_objects
from concept_ranker import trajectory624
from concept_road.field_road import FieldRoad
from concept_road.limits6251 import JunctionLimits, observed_direction
from concept_road.map_semantics618 import SemanticMap
from concept_road.rear_road import RearRoad
from concept_road.runtime_map import RuntimeMap, lane_sections
from concept_road.vision import CX, CY, FOCAL, HEIGHT


def _record_stage(timings, stage, start):
    end = perf_counter_ns()
    timings[stage] = (end - start) / 1e6
    return end


class Drive:
    def __init__(self, pace=3.4, tracking_controller='pure_pursuit'):
        self.tracking_controller = validate_mode(tracking_controller)
        self.observer = RearRoad()
        self.tracker = Occupancy617()
        self.mapper = RuntimeMap()
        self.semantics = SemanticMap((FOCAL, CX, CY, HEIGHT))
        self.limits = JunctionLimits()
        self.field_road = FieldRoad()
        self.planner = MarkedPlanner(dict(pace=pace, uncertainty=1., yaw_limit=1.55,
                                          tracking_controller=self.tracking_controller))
        self.builder = RankedBuilder(BacktrackSteer(self.planner))
        weights = Path(trajectory624.__file__).with_name('model-h12.json')
        self.ranker = trajectory624.TrajectorySelector(weights, pace)
        self.set_pace(pace)
        self.previous_time = None
        self.previous_motion = None
        self.previous_field_edge = None
        self.observed_openings = []

    def set_pace(self, pace):
        if not np.isfinite(pace) or not 0. < pace <= 6.:
            raise ValueError('target speed must be greater than zero and at most 6 m/s')
        self.planner.p['pace'] = float(pace)
        self.ranker.pace = float(pace)

    def step(self, image, raw, lane, det, sensor, timings=None):
        """Advance one frame and optionally record wall-clock stage durations in ms."""
        if timings is not None:
            stage_start = perf_counter_ns()
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
        if timings is not None:
            stage_start = _record_stage(timings, 'input_prepare', stage_start)
        road, _, heading = self.observer.read(image, raw, lane, boxes, motion_speed, motion_yaw, dt)
        evidence = self.observer.last
        if timings is not None:
            stage_start = _record_stage(timings, 'road_observation', stage_start)
        field_ground = sensor.get('field_ground_projection', False)
        measurements = []
        groups = instances(det, image.shape, image, field_ground=field_ground, measurements=measurements)
        objects = self.tracker.update(groups, motion_speed, motion_yaw, dt,
                                      minimum_body_z=float(road[0, 0]) if len(road) else None, image=image)
        if timings is not None:
            stage_start = _record_stage(timings, 'obstacle_tracking', stage_start)
        parts = lane_sections(evidence, raw, (FOCAL, CX, CY, HEIGHT))
        snapshot = self.mapper.update(sensor, parts, objects, self.tracker.last)
        semantic = self.semantics.update(image, raw, lane, boxes, evidence, snapshot)
        if timings is not None:
            stage_start = _record_stage(timings, 'mapping_semantics', stage_start)
        guidance = np.asarray(evidence.get('lane_geometry_rows', [])).reshape(-1, 4)
        road, edges = self.limits.update(road, evidence['tracks'], semantic['lines'], motion_speed, motion_yaw, dt)
        heading, reference = observed_direction(road, heading, evidence['tracks'], edges)
        evidence['circuit_edges'] = edges
        if timings is not None:
            stage_start = _record_stage(timings, 'road_limits', stage_start)
        launch_evidence = None
        if field_ground:
            near_rows = evidence.get('nearfield_rows', [])
            scene = self.field_road.update(image, raw, lane, road, heading, motion_speed, motion_yaw, dt,
                                           nearfield_rows=near_rows, marking_order=edges.get('ordered_paint'),
                                           visible_lines=semantic['lines'], openings=semantic['openings'])
            road = intersect_confirmed_edges(scene.road, road, edges['edges'])
            heading, guidance, reference = scene.heading, scene.guidance, scene.reference
            # A contradictory physical limit invalidates this depth and everything
            # beyond it. Do not interpolate the planner through the missing road.
            heading = heading[:len(road)]
            previous_edge, self.previous_field_edge = self.previous_field_edge, None
            edge = None
            if len(road):
                guidance = guidance[(guidance[:, 0] >= road[0, 0]) & (guidance[:, 0] <= road[-1, 0])]
                if reference is not None:
                    observed = scene.audit['observed_bounds']
                    self.previous_field_edge = fit_edge(road, observed, include_extent=True)
                    edge = self.previous_field_edge or corroborated_previous_edge(
                        previous_edge, road, observed, motion_speed, motion_yaw, dt)
                    reference['_field_edge'] = edge
            else:
                guidance, reference = np.empty((0, 4)), None
            source = 'camera' if self.previous_field_edge is not None else 'previous_fit_confirmed_in_camera'
            scene.audit['edge_preview'] = preview_audit(edge, source, yaw, physical_applied_command()[1])
            near_visibility = [row['z'] for row in near_rows
                               if row['source'] == 'historical_material_observation']
            provisional = launch_objects(det, objects, measurements)
            launch_evidence = dict(stamp_s=float(sensor['t']), applied_speed_mps=physical_applied_command()[0],
                                   observed_near_z=near_visibility, visible_z=scene.audit.get('visible_z'),
                                   obstacle_clear=provisional is not None)
        else:
            self.previous_field_edge = None
            if set(edges['edges']) == {'left', 'right'}:
                guidance = np.empty((0, 4))
            if reference is None:
                reference = build_reference(evidence)
        if timings is not None:
            stage_start = _record_stage(timings, 'field_road_and_launch', stage_start)
        if self.tracking_controller == 'lya':
            if field_ground:
                # Visible_z ends at the first missing direct ground section.
                span = scene.audit.get('visible_z')
            else:
                # Frozen simulator camera uses the canonical mounting height.
                # Exclude blind footprint/motion-memory samples under the lens.
                nearest = HEIGHT*FOCAL/(image.shape[0]-1-CY)
                span = [max(nearest, road[0,0]), road[-1,0]] if len(road) else None
            start = first_lya_depth(span[0], min(span[1], road[-1,0])) if span and len(road) else None
            self.planner.p.update(lya_first_depth=start, lya_odom_heading_right_rad=float(snapshot['pose'][2]))
        obs = RecoveryReference(road, objects, speed, yaw, dt, heading, guidance,
                                evidence['rear_history']['observed_limit_m'],
                                reference=reference)
        if timings is not None:
            stage_start = _record_stage(timings, 'reference_prepare', stage_start)
        initial_reference = 0.
        if field_ground and self.builder.planner.phase == 'forward':
            initial_reference = sensor.get('initial_ramp_reference_mps', 0.)
        self.planner.p['initial_ramp_reference_mps'] = initial_reference
        features, allowed, baseline = self.builder.prepare(obs)
        if timings is not None:
            stage_start = _record_stage(timings, 'candidate_prepare', stage_start)
        pick, ranking = baseline, None
        if self.builder.network_allowed:
            audit = self.builder.planner.last['audit']
            if field_ground and provisional is not None and len(provisional):
                launch_evidence['check_localized_cones'] = partial(
                    launch_clearance, provisional, obs, self.planner.pending, audit)
            pick, ranking = self.ranker.rank(features, allowed, baseline, audit, obs,
                                            **({'launch_evidence': launch_evidence} if field_ground else {}))
        if timings is not None:
            stage_start = _record_stage(timings, 'ranking', stage_start)
        control = self.builder.command(pick)
        if timings is not None:
            stage_start = _record_stage(timings, 'control_select', stage_start)
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
        if control[0] == 0:
            owner = 'stop'
        elif control[0] < 0:
            owner = 'reverse'
        elif detail.get('contingency_execution') or pick is None or detail is not self.planner.last:
            owner = 'contingency'
        elif pick == audit.get('edge_candidate_index'):
            owner = 'edge_recovery'
        else:
            owner = 'candidate'
        tracking = controller_record(self.tracking_controller, control, owner, self.planner.tracking_frame, pick)
        result = dict(tracking_controller=tracking, selected_index=None if pick is None else int(pick),
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
        if timings is not None:
            _record_stage(timings, 'decision_pack', stage_start)
        return result


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
    """True only when launch needs no additional cone check."""
    provisional = launch_objects(detections, objects, measurements)
    return provisional is not None and not len(provisional)
