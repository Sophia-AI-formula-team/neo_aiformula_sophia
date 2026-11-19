"""Accept one synchronized perception result and advance the frozen V6.26 driver once.

All geometry here uses metres forward/right. Sensor gyro and output yaw in this
module are clockwise-positive; the ROS adapter performs the two sign conversions.
"""
from pathlib import Path

import numpy as np

from concept_candidates.backtrack_forward617 import BacktrackSteer
from concept_candidates.curve_progress import build_reference
from concept_candidates.planner625 import MarkedPlanner
from concept_candidates.ranked_builder617 import RankedBuilder
from concept_candidates.rear_reference617 import RecoveryReference
from concept_obstacles.obstacles617 import Occupancy617, instances
from concept_ranker import trajectory624
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
        if set(edges['edges']) == {'left', 'right'}:
            guidance = np.empty((0, 4))
        obs = RecoveryReference(road, objects, speed, yaw, dt, heading, guidance,
                                evidence['rear_history']['observed_limit_m'],
                                reference=reference if reference is not None else build_reference(evidence))
        features, allowed, baseline = self.builder.prepare(obs)
        pick, ranking = baseline, None
        if self.builder.network_allowed:
            pick, ranking = self.ranker.rank(features, allowed, baseline, self.builder.planner.last['audit'], obs)
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
                    reason=detail.get('reason', ''), recovery=detail.get('recovery', ''),
                    ranking_active=ranking is not None, road_rows=road.tolist(), heading=heading.tolist(),
                    **({'obstacle_measurements': measurements} if field_ground else {}),
                    objects=objects.tolist(), tracks=self.tracker.last['tracks'], speed_estimate_mps=float(speed),
                    rejected=audit.get('rejected', {}), edges=edges['edges'], lines=semantic['lines'],
                    map=snapshot, map_semantics={k: semantic[k] for k in ('changed_markings','changed_gaps','changed_repairs')})
