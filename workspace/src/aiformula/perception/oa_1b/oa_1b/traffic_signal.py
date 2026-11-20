"""Traffic-light outputs for the OA 1B experiment. Positions use source-image pixels."""
import math

import cv2
import numpy as np


def overlap(a, b):
    w = max(0., min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    i = w * h
    return i / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - i)


def bounds(box, shape):
    if len(box) != 4 or not all(math.isfinite(v) for v in box):
        return None
    if box[2]<=box[0] or box[3]<=box[1]:
        return None
    h, w = shape[:2]
    x, y = max(0, math.floor(box[0])), max(0, math.floor(box[1]))
    r, b = min(w, math.ceil(box[2])), min(h, math.ceil(box[3]))
    return (x, y, r, b) if r > x and b > y else None


def red_mask(tile):
    hue, saturation, value = cv2.split(cv2.cvtColor(tile, cv2.COLOR_BGR2HSV))
    return (((hue <= 9) | (hue >= 170)) & (saturation >= 75) & (value >= 65)).astype('uint8')


def red_area(image, boxes):
    """Same HSV and floor/ceil pixel convention as field AV314, counting each pixel once."""
    bounds_list = [bounds(box, image.shape) for box in boxes]
    unique_boxes = [v for v in bounds_list if v is not None]
    if not unique_boxes:
        return 0
    x = min(box[0] for box in unique_boxes)
    y = min(box[1] for box in unique_boxes)
    r = max(box[2] for box in unique_boxes)
    b = max(box[3] for box in unique_boxes)
    mask = red_mask(image[y:b, x:r])
    if len(unique_boxes) == 1:
        return int(mask.sum())
    covered = np.zeros(mask.shape, dtype=np.uint8)
    for left, top, right, bottom in unique_boxes:
        covered[top-y:bottom-y, left-x:right-x] = 1
    return int((mask & covered).sum())


def current_face(image, box):
    x, y, r, b = bounds(box, image.shape)
    mask = red_mask(image[y:b, x:r])
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    for _, _, width, height, area in stats[1:]:
        if area >= 15 and .70 <= width / height <= 1.70 and area >= .62 * width * height:
            return True
    return False


class SignalTracker:
    """An accepted lamp may be rechecked only around an actual, confirmed stop."""
    def __init__(self):
        self.last_stamp = self.last_ros_now = self.shape = None
        self.status_at = self.odom_at = -math.inf
        self.waiting = False
        self.position = None
        self.speed = math.inf
        self.reset()

    def reset(self):
        self.last_native = None
        self.native_stamp = 0
        self.native_count = 0
        self.anchor = self.park_position = None
        self.last_wait = -math.inf
        self.consecutive = 0

    def status(self, payload, now):
        mode = payload.get('traffic_mode')
        if mode not in ('normal','prepare','wait') or payload.get('traffic_reason') == 'manual_motion_resume':
            self.reset()
        self.waiting = (mode == 'wait' and payload.get('red_stop_required') is True
                        and payload.get('traffic_stop_committed') is True)
        self.status_at = now

    def odometry(self, x, y, speed, now):
        if not all(math.isfinite(v) for v in (x,y,speed)):
            self.reset()
            self.odom_at = -math.inf
            return
        self.position, self.speed, self.odom_at = (x,y), abs(speed), now

    def accept_stamp(self, stamp, ros_now):
        if self.last_ros_now is not None and ros_now < self.last_ros_now-500_000_000:
            self.reset()
            self.last_stamp = None
            self.status_at = self.odom_at = -math.inf
        self.last_ros_now = ros_now
        if stamp <= 0 or not 0 <= ros_now - stamp <= 500_000_000:
            self.reset()
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.consecutive = 0
            return False
        if self.last_stamp is not None and stamp - self.last_stamp >= 250_000_000:
            self.reset()
        self.last_stamp = stamp
        return True

    def update_anchor(self, image, lights, state, stamp, now):
        if state=='green' or now-self.status_at>.65 or now-self.odom_at>.5:
            self.reset()
            return
        if self.anchor is not None:
            if self.waiting:
                self.last_wait = now
            if now-self.last_wait>2.5 or math.dist(self.position, self.park_position)>3.0:
                self.reset()
            else:
                return
        red = [v for v in lights if v.get('traffic_light_state')=='red' and bounds(v['box_xyxy'],image.shape)]
        if state=='red' and red:
            box = tuple(max(red, key=lambda v:v['score'])['box_xyxy'])
            continuous = (self.last_native is not None and stamp-self.native_stamp < 500_000_000
                          and overlap(box, self.last_native)>=.50)
            self.native_count = self.native_count+1 if continuous else 1
            self.last_native, self.native_stamp = box, stamp
        if (self.waiting and self.speed < .05 and self.native_count>=2
                and stamp-self.native_stamp < 500_000_000):
            self.anchor, self.park_position = self.last_native, self.position
            self.last_wait = now

    def correct(self, image, original, candidates, stamp, now, field_colour):
        """Candidates are current-frame class 10 (box, score) from the detector ROI head."""
        if self.shape != image.shape[:2]:
            self.reset()
            self.shape = image.shape[:2]
        lights = [v for v in original['detections'] if v['class_id']==10]
        state = original['traffic_light_state']
        self.update_anchor(image, lights, state, stamp, now)
        matching = []
        if self.anchor is not None and current_face(image, self.anchor):
            for box, score in candidates:
                if not math.isfinite(score) or score < .30 or bounds(box, image.shape) is None or overlap(self.anchor,box)<.50:
                    continue
                colour = field_colour(image, box)
                if colour.label=='red' and colour.counts['red']>=15:
                    matching.append((score,box))
        self.consecutive = self.consecutive+1 if matching else 0
        corrected = dict(original)
        rescued = state=='unknown' and self.consecutive>=2
        if rescued:
            score, box = max(matching, key=lambda v:v[0])
            lamp = dict(box_xyxy=box, score=score, class_id=10, class_name='traffic light',
                        color=None, semantic_label='red_traffic_light', traffic_light_state='red',
                        source='oa_1b_same_light')
            corrected['detections'] = original['detections'] + [lamp]
            corrected['traffic_light_state'] = 'red'
            lights.append(lamp)
        corrected['red_pixels_count'] = red_area(image, [v['box_xyxy'] for v in lights])
        corrected['oa_1b'] = dict(rescued=rescued, tracked=self.anchor is not None,
                                  stock_state=state, stock_red_pixels=original['red_pixels_count'])
        return corrected
