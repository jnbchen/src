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
        self.marker_.scale.x = 0.1
        self.marker_.scale.y = 0.1
        self.marker_.lifetime = rospy.Duration(0.5)

        self.point_ = Marker()
        self.point_.action = Marker.ADD
        self.point_.type = Marker.POINTS
        self.point_.scale.x = 0.1
        self.point_.scale.y = 0.1
        self.point_.pose.orientation.w = 1.0
        self.point_.lifetime = rospy.Duration(0.5)

        self.text_ = Marker()
        self.text_.action = Marker.ADD
        self.text_.type = Marker.TEXT_VIEW_FACING
        self.text_.scale.x = 0.1
        self.text_.scale.y = 0.1
        self.text_.pose.orientation.w = 1.0
        self.text_.lifetime = rospy.Duration(0.5)

        self.point_.color.r = self.text_.color.r = 1.0
        self.point_.color.g = self.text_.color.g = 1.0
        self.point_.color.b = self.text_.color.b = 1.0
        self.point_.color.a = self.text_.color.a = 1.0

        self.ssr_array_ = MarkerArray()
        self.ars_array_ = MarkerArray()

        self.pub_ssr_ = rospy.Publisher('/can_dumper_node/can_ssr', MarkerArray, queue_size=20)
        self.pub_ars_ = rospy.Publisher('/can_dumper_node/can_ars', MarkerArray, queue_size=20)

    def callback_ssr(self, msg):
        for i in range(len(msg.radar_array)):
            track = msg.radar_array[i]
            #print('device: %s, id: %d') %(track.radar_type, track.id) 
            if not track.is_object:
                p = Point()
                p.x = track.distance_x
                p.y = track.distance_y
                self.point_.id = track.id
                self.point_.header.stamp = self.text_.header.stamp = rospy.get_rostime()
                self.point_.header.frame_id = track.radar_type
                self.point_.ns = track.radar_type
                self.point_.points = [p]
                self.ssr_array_.markers.append(self.point_)

                self.text_.id = track.id + 1000
                self.text_.header.frame_id = track.radar_type
                self.text_.ns = track.radar_type + '-id'
                self.text_.pose.position = p
                self.text_.text = 'id-' + str(track.id) + '/Vx-' + str(relative_velocity_x) + '/Vy-' + str(relative_velocity_x)
                self.ssr_array_.markers.append(self.text_)

        self.pub_ssr_.publish(self.ssr_array_)
        self.ssr_array_.markers[:] = []

    def callback_ars(self, msg): 
        for i in range(len(msg.radar_array)):
            track = msg.radar_array[i]
            if track.is_object and track.probolity_of_exist > 2:
                self.marker_.id = track.id
                self.marker_.ns = track.radar_type
                self.marker_.header.frame_id = track.radar_type
                self.marker_.header.stamp = self.text_.header.stamp = rospy.get_rostime()
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
                self.ars_array_.markers.append(self.marker_)

                self.text_.id = track.id + 1000
                self.text_.header.frame_id = track.radar_type
                self.text_.ns = track.radar_type + '-id'
                self.text_.pose.position.x = track.distance_x
                self.text_.pose.position.y = track.distance_y
                self.text_.text = 'id-' + str(track.id) + '/Vx-' + str(relative_velocity_x) + '/Vy-' + str(relative_velocity_x)
                self.ars_array_.markers.append(self.text_)

        self.pub_ars_.publish(self.ars_array_)
        self.ars_array_.markers[:] = []

    def start(self):
        rospy.Subscriber('/can_dumper_node/can1_data', RadarArray, self.callback_ssr, queue_size=1000)
        rospy.Subscriber('/can_dumper_node/can2_data', RadarArray, self.callback_ssr, queue_size=1000)
        rospy.Subscriber('/can_dumper_node/can5_data', RadarArray, self.callback_ars, queue_size=1000)
        rate = rospy.Rate(30)
        rate.sleep()
        rospy.spin()
