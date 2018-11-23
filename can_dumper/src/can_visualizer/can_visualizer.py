#!/usr/bin/env python

import rospy, math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from can_dumper.msg import *

class can_visualizer(object):
    def __init__(self):
        self.marker_ = Marker()
        self.marker_.action = Marker.ADD
        self.marker_.type = Marker.LINE_STRIP
        self.marker_.scale.x = 0.5
        self.marker_.scale.y = 0.5
        self.marker_.lifetime = rospy.Duration(0.5)

        self.point_ = Marker()
        self.point_.action = Marker.ADD
        self.point_.type = Marker.POINTS
        self.point_.scale.x = 0.5
        self.point_.scale.y = 0.5
        self.point_.pose.orientation.w = 1.0
        self.point_.lifetime = rospy.Duration(0.5)

        self.point_.color.r = 1.0
        self.point_.color.g = 1.0
        self.point_.color.b = 1.0
        self.point_.color.a = 1.0

        self.ContiARS_ = MarkerArray()

        self.pub_track_ = rospy.Publisher('/can_dumper_node/can_track', Marker, queue_size=20)
        self.pub_object_ = rospy.Publisher('/can_dumper_node/can_object', MarkerArray, queue_size=20)

    def callback(self, msg):
        for i in range(len(msg.radar_array)):
            track = msg.radar_array[i]
            #print('device: %s, id: %d') %(track.radar_type, track.id) 
            if track.is_object:
                self.track_object(track)
            else:
                self.track_point(track)

        self.pub_track_.publish(self.point_)
        self.pub_object_.publish(self.ContiARS_)

        self.point_.points[:] = []
        self.ContiARS_.markers[:] = []

    def track_object(self, track): 
        self.marker_.id = track.id
        self.marker_.ns = track.radar_type
        self.marker_.header.frame_id = track.radar_type
        w = track.object_size_x
        l = track.object_size_y
        p1 = p2 = p3 = p4 = Point()
        p1.x = -w
        p1.y = -l
        p2.x = -w
        p2.y = l
        p3.x = w
        p3.x = l
        p4.x = w
        p4.y = -l
        self.marker_.points[:]=[]
        self.marker_.points.append(p1)
        self.marker_.points.append(p2)
        self.marker_.points.append(p3)
        self.marker_.points.append(p4)
        self.marker_.points.append(p1)
        self.marker_.pose.position.x = track.distance_x
        self.marker_.pose.position.y = track.distance_y
        self.marker_.pose.orientation.z = math.sin(0.5*track.angle)
        self.marker_.pose.orientation.w = math.cos(0.5*track.angle)
        self.marker_.color.r = track.r
        self.marker_.color.g = track.g
        self.marker_.color.b = track.b
        self.marker_.color.a = track.a
        self.ContiARS_.markers.append(self.marker_)

    def track_point(self, track):
        p = Point()
        p.x = track.distance_x
        p.y = track.distance_y
        self.point_.id = track.id
        self.point_.header.stamp = rospy.get_rostime()
        self.point_.header.frame_id = track.radar_type
        self.point_.ns = track.radar_type
        self.point_.points.append(p)

    def start(self):
        rospy.Subscriber('/can_dumper_node/can_data', RadarArray, self.callback, queue_size=20)
        rate = rospy.Rate(30)
        rate.sleep()
        rospy.spin()
