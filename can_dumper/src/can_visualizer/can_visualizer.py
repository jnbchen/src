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
        self.marker_.scale.x = 0.3
        self.marker_.scale.y = 0.3
        self.marker_.scale.z = 0.3
        self.marker_.lifetime = rospy.Duration(0.5)

        self.point_ = Marker()
        self.point_.action = Marker.ADD
        self.point_.type = Marker.POINTS
        self.point_.scale.x = 0.5
        self.point_.scale.y = 0.5
        self.point_.pose.orientation.w = 1.0
        self.point_.lifetime = rospy.Duration(0.5)

        self.text_ = Marker()
        self.text_.action = Marker.ADD
        self.text_.type = Marker.TEXT_VIEW_FACING
        self.text_.scale.x = 1
        self.text_.scale.y = 1
        self.text_.scale.z = 1.5
        self.text_.pose.orientation.w = 1.0
        self.text_.lifetime = rospy.Duration(0.5)

        self.class_ = Marker()
        self.class_.action = Marker.ADD
        self.class_.type = Marker.TEXT_VIEW_FACING
        self.class_.scale.x = 1
        self.class_.scale.y = 1
        self.class_.scale.z = 1.5
        self.class_.pose.orientation.w = 1.0
        self.class_.lifetime = rospy.Duration(0.5)

        self.park_ = Marker()
        self.park_.id = 1
        self.park_.header.frame_id = 'UltraSonic'
        self.park_.action = Marker.ADD
        self.park_.type = Marker.LINE_STRIP
        self.park_.scale.x = 1
        self.park_.scale.y = 1
        self.park_.scale.z = 1.5
        self.park_.pose.orientation.w = 1.0
        self.park_.lifetime = rospy.Duration(0.5)

        self.point_.color.r = self.text_.color.r = self.park_.color.r = 1.0
        self.point_.color.g = self.text_.color.g = self.park_.color.r = 1.0
        self.point_.color.b = self.text_.color.b = self.park_.color.r = 1.0
        self.point_.color.a = self.text_.color.a = self.park_.color.r = 1.0

        self.ssr_array_ = MarkerArray()
        self.ars_array_ = MarkerArray()

        self.pub_ssr_ = rospy.Publisher('/can_dumper_node/can_ssr', MarkerArray, queue_size=100)
        self.pub_ars_ = rospy.Publisher('/can_dumper_node/can_ars', MarkerArray, queue_size=100)
        self.pub_ultra_ = rospy.Publisher('/can_dumper_node/can_ultra', Marker, queue_size=100)

	self.topic_can0 = rospy.get_param('~topic_can0', False)
        self.topic_can1 = rospy.get_param('~topic_can1', False)
        self.topic_can2 = rospy.get_param('~topic_can2', False)
	self.topic_can5 = rospy.get_param('~topic_can5', False)

    def callback_ssr(self, msg):
        for i in range(len(msg.radar_array)):
            track = msg.radar_array[i]
            #print('device: %s, id: %d') %(track.radar_type, track.id) 
            if not track.is_object and track.roll_count > 2:
                p = Point()
                p.x = track.distance_x
                p.y = track.distance_y
                self.point_.id = track.id
                self.point_.header.stamp = self.text_.header.stamp = rospy.get_rostime()
                self.point_.header.frame_id = track.radar_type
                self.point_.ns = track.radar_type
                self.point_.points = [p]
                self.ssr_array_.markers.append(self.point_)

                self.text_.id = track.id + 100000
                self.text_.header.frame_id = track.radar_type
                self.text_.ns = track.radar_type
                self.text_.pose.position = p
                self.text_.text = 'id-' + str(track.id) + '/Vx-' + str(track.relative_velocity_x) + '/Vy-' + str(track.relative_velocity_x)
                self.ssr_array_.markers.append(self.text_)

        self.pub_ssr_.publish(self.ssr_array_)
        self.ssr_array_.markers[:] = []

    def callback_ars(self, msg):
        for i in range(len(msg.radar_array)):
            track = msg.radar_array[i]
            if track.is_object:
                #print(track.probolity_of_exist)
                self.marker_.id = track.id
                self.marker_.ns = track.radar_type
                #print(track.radar_type)
                self.marker_.header.frame_id = track.radar_type
                self.marker_.header.stamp = self.text_.header.stamp = rospy.get_rostime()
                w = track.object_size_x
                l = track.object_size_y
                p1 = Point()
                p2 = Point()
                p3 = Point()
                p4 = Point()
                p1.x = 0.5*w
                p1.y = 0.5*l
                p2.x = -0.5*w
                p2.y = 0.5*l
                p3.x = -0.5*w
                p3.y = -0.5*l
                p4.x = 0.5*w
                p4.y = -0.5*l
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

                self.class_.id = track.id + 100000
                self.class_.header.frame_id = track.radar_type
                self.class_.ns = track.radar_type
                self.class_.pose.position.x = track.distance_x
                self.class_.pose.position.y = track.distance_y
                self.class_.text =  track.name
                self.class_.color.r = track.r
                self.class_.color.g = track.g
                self.class_.color.b = track.b
                self.class_.color.a = track.a
                self.ars_array_.markers.append(self.class_)

        self.pub_ars_.publish(self.ars_array_)
        self.ars_array_.markers[:] = []

    def callback_ultra(self, msg):
        self.park_.id += 1
        self.park_.header.stamp = rospy.get_rostime()
        p1 = Point()
        p2 = Point()
        p3 = Point()
        p4 = Point()
        p1.x = 0.5*msg.lenth_l
        p1.y = 0.5*msg.deepth_l
        p2.x = -0.5*msg.lenth_l
        p2.y = 0.5*msg.deepth_l
        p3.x = -0.5*msg.lenth_l
        p3.y = -0.5*msg.deepth_l
        p4.x = 0.5*msg.lenth_l
        p4.y = -0.5*msg.deepth_l
        self.park_.points.append(p1)
        self.park_.points.append(p2)
        self.park_.points.append(p3)
        self.park_.points.append(p4)
        self.park_.points.append(p1)
        self.park_.pose.position.x = 0.5*msg.corner_A_x + 0.5*msg.corner_B_x
        self.park_.pose.position.y = 0.5*msg.corner_A_y + 0.5*msg.corner_B_y + 0.5* msg.deepth_l
        self.pub_ultra_.publish(self.park_)

        self.park_.id += 1
        p1.x = 0.5*msg.lenth_r
        p1.y = 0.5*msg.deepth_r
        p2.x = -0.5*msg.lenth_r
        p2.y = 0.5*msg.deepth_r
        p3.x = -0.5*msg.lenth_r
        p3.y = -0.5*msg.deepth_r
        p4.x = 0.5*msg.lenth_r
        p4.y = -0.5*msg.deepth_r
        self.park_.points.append(p1)
        self.park_.points.append(p2)
        self.park_.points.append(p3)
        self.park_.points.append(p4)
        self.park_.points.append(p1)
        self.park_.pose.position.x = 0.5*msg.corner_C_x + 0.5*msg.corner_D_x
        self.park_.pose.position.y = 0.5*msg.corner_C_y + 0.5*msg.corner_D_y + 0.5* msg.deepth_r
        self.pub_ultra_.publish(self.park_)

    def start(self):
	if self.topic_can1:
            rospy.Subscriber('/can_dumper_node/can1_data', RadarArray, self.callback_ssr)
        if self.topic_can2:
            rospy.Subscriber('/can_dumper_node/can2_data', RadarArray, self.callback_ssr)
        if self.topic_can5:
            rospy.Subscriber('/can_dumper_node/can5_data', RadarArray, self.callback_ars)
        if self.topic_can0:
            rospy.Subscriber('/can_dumper_node/can0_data', UltraSonic, self.callback_ultra,)
        rospy.spin()
