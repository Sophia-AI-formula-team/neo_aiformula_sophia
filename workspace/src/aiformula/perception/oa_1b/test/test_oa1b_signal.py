import unittest
from types import SimpleNamespace

import numpy as np
from oa_1b.traffic_signal import SignalTracker


class SignalTests(unittest.TestCase):
    def setUp(self):
        self.tracker=SignalTracker()
        self.on=np.zeros((32,32,3),np.uint8)
        self.on[10:15,10:15,2]=255
        self.off=np.zeros_like(self.on)
        self.box=(8.,8.,18.,18.)
        self.base={'detections':[],'traffic_light_state':'unknown','red_pixels_count':0}
        self.native={'class_id':10,'box_xyxy':self.box,'score':.6,'traffic_light_state':'red'}
        self.sec=0.9

    def frame(self, status='wait', native=False, lit=True, score=.40, gap=.075, speed=0., x=0., box=None):
        self.sec+=gap
        track=self.tracker
        track.status({'traffic_mode':status,'red_stop_required':status=='wait',
                      'traffic_stop_committed':status=='wait'},self.sec)
        track.odometry(x,0.,speed,self.sec)
        stamp=int(self.sec*1e9)
        self.assertTrue(track.accept_stamp(stamp,stamp+50_000_000))
        result=dict(self.base)
        if native:
            result.update(detections=[self.native],traffic_light_state='red',red_pixels_count=25)
        colour=SimpleNamespace(label='red' if lit else 'unknown',counts={'red':25 if lit else 0})
        return track.correct(self.on if lit else self.off,result,[(box or self.box,score)],stamp,self.sec,
                             lambda *_:colour)

    def test_acquire_only_from_actual_confirmed_stop_and_native_reds(self):
        for _ in range(3):
            self.assertEqual(self.frame()['traffic_light_state'],'unknown')
        for _ in range(3):
            self.assertFalse(self.frame(status='normal',native=True)['oa_1b']['tracked'])
        self.assertEqual(self.frame()['traffic_light_state'],'unknown')
        self.assertTrue(self.frame()['oa_1b']['rescued'])
        self.assertEqual(self.frame(lit=False)['traffic_light_state'],'unknown')
        self.assertEqual(self.frame()['traffic_light_state'],'unknown')
        self.assertTrue(self.frame()['oa_1b']['rescued'])
        self.assertEqual(self.frame(score=.28)['traffic_light_state'],'unknown')
        self.assertEqual(self.frame(box=(20,20,30,30))['traffic_light_state'],'unknown')
        self.assertFalse(self.frame(status='normal',x=4.)['oa_1b']['tracked'])

    def test_duplicate_gap_and_reset_require_fresh_confirmation(self):
        self.frame(native=True)
        self.frame(native=True)
        self.assertTrue(self.frame()['oa_1b']['rescued'])
        stamp=int(self.sec*1e9)
        self.assertFalse(self.tracker.accept_stamp(stamp,stamp))
        self.assertEqual(self.frame()['traffic_light_state'],'unknown')
        self.assertFalse(self.frame(gap=.30)['oa_1b']['tracked'])
        self.frame(native=True)
        self.frame(native=True)
        self.tracker.status(dict(traffic_mode='normal',traffic_reason='manual_motion_resume'),self.sec)
        self.assertFalse(self.frame()['oa_1b']['tracked'])
        self.assertFalse(self.tracker.accept_stamp(stamp,stamp+600_000_000))

    def test_unique_red_does_not_modify_stock(self):
        stock=dict(detections=[self.native,self.native],traffic_light_state='red',red_pixels_count=50)
        fixed=self.tracker.correct(self.on,stock,[],10**9,1.,None)
        self.assertEqual(fixed['red_pixels_count'],25)
        self.assertEqual(stock['red_pixels_count'],50)

    def test_moving_car_cannot_arm(self):
        for _ in range(3):
            self.assertFalse(self.frame(native=True,speed=1.)['oa_1b']['tracked'])
        self.assertFalse(self.frame(speed=.1)['oa_1b']['tracked'])

    def test_green_releases_association_and_ros_clock_restart_drops_old_history(self):
        self.frame(native=True)
        self.frame(native=True)
        self.assertTrue(self.frame()['oa_1b']['rescued'])
        green=dict(self.base,traffic_light_state='green')
        fixed=self.tracker.correct(self.off,green,[],int(self.sec*1e9),self.sec,None)
        self.assertEqual(fixed['traffic_light_state'],'green')
        self.assertFalse(fixed['oa_1b']['tracked'])
        self.frame(native=True)
        self.frame(native=True)
        self.assertIsNotNone(self.tracker.anchor)
        self.assertTrue(self.tracker.accept_stamp(20_000_000,50_000_000))
        self.assertIsNone(self.tracker.anchor)


if __name__=='__main__':
    unittest.main()
