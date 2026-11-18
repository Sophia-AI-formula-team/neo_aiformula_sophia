"""Publish displays from the same stamped decisions the vehicle used."""
from collections import OrderedDict
import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import ObservedMap, body_point, memory_to_body, pixel

FRAME = 'base_footprint'
COLORS = {'road': (.20,.43,.53,.65), 'paint': (1.,.54,.12,1.),
          'chosen': (.97,.29,.63,1.), 'allowed': (.27,.76,.64,.32),
          'blocked': (.52,.56,.59,.14), 'cone': (1.,.45,.09,1.),
          'footprint': (.76,.82,.82,1.), 'ego': (.14,.49,.68,.75),
          'adjacent': (.19,.60,.43,.60)}


def stamp_key(stamp):
    return int(stamp.sec), int(stamp.nanosec)


def remember(buffer, key, value):
    buffer[key] = value
    if len(buffer)>12:
        buffer.popitem(last=False)


def draw_path(image, path, color, width):
    uv = [pixel(p) for p in path]
    for a,b in zip(uv,uv[1:]):
        if a and b:
            ok,p,q=cv2.clipLine((0,0,image.shape[1],image.shape[0]),a,b)
            if ok:cv2.line(image,p,q,color,width,cv2.LINE_AA)


class Scene:
    def __init__(self, stamp):
        self.stamp = stamp
        self.markers = []
        self.add('clear', Marker.LINE_STRIP, [], 'chosen', .02).action = Marker.DELETEALL

    def add(self, ns, shape, points, color, size, z=.05):
        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp.sec, marker.header.stamp.nanosec = self.stamp
        marker.ns, marker.id, marker.type = ns, len(self.markers), shape
        marker.pose.orientation.w = 1.
        marker.color.r,marker.color.g,marker.color.b,marker.color.a=COLORS[color]
        marker.scale.x = float(size)
        marker.scale.y = float(size)
        marker.scale.z = float(size if shape==Marker.SPHERE_LIST else .025)
        marker.points = [Point(x=float(x), y=float(y), z=z) for x,y in points]
        self.markers.append(marker)
        return marker


class Visualization(Node):
    def __init__(self):
        super().__init__('concept_visualization')
        self.bridge= CvBridge()
        self.images, self.decisions = OrderedDict(), OrderedDict()
        self.map = ObservedMap()
        self.last_map = -math.inf
        self.view = self.create_publisher(Image,'/concept/view/camera',qos_profile_sensor_data)
        self.local = self.create_publisher(MarkerArray,'/concept/view/local',2)
        self.memory = self.create_publisher(MarkerArray,'/concept/view/map',2)
        self.create_subscription(Image,'/concept/perception/overlay',self.on_image,qos_profile_sensor_data)
        self.create_subscription(String,'/concept/decision',self.on_decision,10)

    def on_image(self,msg):
        if not self.view.get_subscription_count():
            return
        key = stamp_key(msg.header.stamp)
        remember(self.images,key,msg)
        if key in self.decisions:
            self.camera(msg,self.decisions[key])
        else:
            self.view.publish(msg)

    def camera(self,msg,d):
        image=self.bridge.imgmsg_to_cv2(msg,'bgr8').copy()
        if image.shape[:2] == (360,640):
            for points in (d.get('edges') or {}).values():
                draw_path(image,points,(37,157,248),2)
            draw_path(image,d.get('selected_path',[]),(170,72,250),4)
        out=self.bridge.cv2_to_imgmsg(np.ascontiguousarray(image),'bgr8')
        out.header=msg.header
        self.view.publish(out)

    def on_decision(self,msg):
        d=json.loads(msg.data)
        if not d.get('stamp'):
            return
        key=d['stamp']['sec'],d['stamp']['nanosec']
        remember(self.decisions,key,d)
        if self.view.get_subscription_count() and key in self.images and key==next(reversed(self.images)):
            self.camera(self.images[key],d)
        if self.local.get_subscription_count():
            self.publish_local(d, key)
        self.map.update(d)
        now=time.monotonic()
        data=d.get('map') or {}
        if (now-self.last_map>=.5 and data.get('pose') is not None
                and self.memory.get_subscription_count()):
            s=Scene(key)
            for color,points in self.map.points(data['pose']).items():
                s.add('memory_'+color,Marker.CUBE_LIST,points,color,.18,z=-.025 if color!='paint' else .01)
            pose = data['pose']
            fixed = [memory_to_body(pose, *o['position']) for o in data.get('static', [])]
            s.add('remembered_cones', Marker.SPHERE_LIST, fixed, 'cone', .36, z=.18)
            for obj in data.get('now', []):
                if obj['motion'] != 'moving':
                    continue
                for predicted in obj.get('ranges', []):
                    # One standard deviation of the estimated obstacle centre.
                    covariance = np.asarray(predicted['covariance'])
                    values, axes = np.linalg.eigh(covariance)
                    theta = np.linspace(0., 2.*np.pi, 25)
                    ellipse = axes @ (np.sqrt(np.maximum(values, 0.))[:,None] *
                                      np.array([np.cos(theta), np.sin(theta)]))
                    points = ellipse.T + np.asarray(predicted['center'])
                    s.add('moving_uncertainty', Marker.LINE_STRIP,
                          [memory_to_body(pose, *p) for p in points], 'cone', .035, z=.16)
            self.memory.publish(MarkerArray(markers=s.markers))
            self.last_map=now

    def publish_local(self, d, key):
        s=Scene(key)
        cv=lambda pts:[body_point(*p[:2]) for p in pts]
        s.add('front_axle',Marker.LINE_STRIP,[(0.,-.30),(0.,.30)],'footprint',.08)
        for row in (d.get('map') or {}).get('parts',[]):
            mesh=[]
            rows=row['rows']
            for a,b in zip(rows,rows[1:]):
                al,ar=body_point(a[0],a[1]),body_point(a[0],a[2])
                bl,br=body_point(b[0],b[1]),body_point(b[0],b[2])
                mesh.extend([al,ar,bl,bl,ar,br])
            m=s.add('lane_'+row['kind'],Marker.TRIANGLE_LIST,mesh,'adjacent' if row['kind']=='adjacent' else 'road',1.,z=0.)
            m.scale.z=1.
        for line in d.get('lines',[]):
            pts=line.get('points',[])
            if len(pts)>1:s.add('paint_'+line.get('kind',''),Marker.LINE_STRIP,cv(pts),'paint',.045)
        for name,pts in (d.get('edges') or {}).items():
            s.add(name,Marker.LINE_STRIP,cv(pts),'paint',.07)
        for item in d.get('candidates',[]):
            pts=item.get('path',[])
            if len(pts)>1:
                s.add('feasible' if item['allowed'] else 'excluded',Marker.LINE_STRIP,cv(pts),
                      'allowed' if item['allowed'] else 'blocked',.018)
        s.add('selected',Marker.LINE_STRIP,cv(d.get('selected_path',[])),'chosen',.105,z=.12)
        for obj in d.get('objects',[]):
            if len(obj)<2:continue
            x,y=body_point(obj[0],obj[1])
            s.add('obstacle',Marker.SPHERE_LIST,[(x,y)],'cone',.36,z=.18)
            if len(obj)>=4 and math.hypot(obj[2],obj[3])>.05:
                s.add('motion_1s',Marker.LINE_STRIP,[(x,y),(x+obj[2],y-obj[3])],'cone',.075,z=.18)
        self.local.publish(MarkerArray(markers=s.markers))


def main(args=None):
    rclpy.init(args=args)
    node=Visualization()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
