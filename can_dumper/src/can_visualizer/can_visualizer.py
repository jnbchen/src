#!/usr/bin/env python

import rospy, math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from can_dumper.msg import *

class can_visualizer(object):
    def __init__(self):
        self.ssr_array_ = MarkerArray()
        self.ars_array_ = MarkerArray()
        self.pub_ssr_ = rospy.Publisher('/can_dumper_node/can_ssr', MarkerArray, queue_size=1)
        self.pub_ars_ = rospy.Publisher('/can_dumper_node/can_ars', MarkerArray, queue_size=1)

        self.topic_can1 = rospy.get_param('~topic_can1', False)
        self.topic_can2 = rospy.get_param('~topic_can2', False)
        self.topic_can5 = rospy.get_param('~topic_can5', False)

    def callback_ssr(self, msg):
        for track in msg.radar_array:
          if track is not None:
            #print('device: %s, id: %d') %(track.radar_type, track.id) 
            if not track.is_object and track.roll_count > 1:
                marker = Marker()
                p = Point()
                txt = Marker()

                p.x = track.pose.x
                p.y = track.pose.y

                marker.action = Marker.ADD
                marker.type = Marker.POINTS
                marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                marker.scale = Vector3(0.5, 0.5, 1)
                marker.pose.orientation.w = 1.0
                marker.lifetime = rospy.Duration(0.5)

                marker.id = track.id
                marker.header.stamp = txt.header.stamp = rospy.get_rostime()
                marker.header.frame_id = txt.header.frame_id = track.radar_type
                marker.ns = txt.ns = track.radar_type
                marker.points = [p]
                self.ssr_array_.markers.append(marker)

                txt.action = Marker.ADD
                txt.type = Marker.TEXT_VIEW_FACING
                txt.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                txt.scale = Vector3(0.5, 0.5, 1)
                txt.pose.orientation.w = 1.0
                txt.lifetime = rospy.Duration(0.5)
                txt.id = track.id + 100000
                txt.pose.position = p
                txt.text = 'id-' + str(track.id) + '/Vx-' + str(track.relative_velocity_x) + '/Vy-' + str(track.relative_velocity_x)
                self.ssr_array_.markers.append(txt)

        self.pub_ssr_.publish(self.ssr_array_)
        self.ssr_array_.markers[:] = []

    def callback_ars(self, msg):
        for track in msg.radar_array:
          if track is not None:
            if track.is_object and track.probolity_of_exist > 2:
                box = Marker()
                name = Marker()

                box.action = Marker.ADD
                box.type = Marker.LINE_STRIP
                box.scale = Vector3(0.5, 0.5, 1)
                box.lifetime = rospy.Duration(0.5)
                #print(track.probolity_of_exist)
                box.id = track.id
                box.ns = name.ns = track.radar_type
                box.header.frame_id = name.header.frame_id = track.radar_type
                box.header.stamp = name.header.stamp = rospy.get_rostime()
                w = track.object_size_x
                l = track.object_size_y
                p1 = Point( 0.5*w,  0.5*l, 0)
                p2 = Point(-0.5*w,  0.5*l, 0)
                p3 = Point(-0.5*w, -0.5*l, 0)
                p4 = Point( 0.5*w, -0.5*l, 0)
                box.points[:] = []
                box.points = [p1, p2, p3, p4, p1]
                box.pose.position.x = track.pose.x
                box.pose.position.y = track.pose.y
                box.pose.orientation.z = math.sin(0.5*track.pose.theta)
                box.pose.orientation.w = math.cos(0.5*track.pose.theta)
                box.color = name.color = track.color
                self.ars_array_.markers.append(box)

                name.action = Marker.ADD
                name.type = Marker.TEXT_VIEW_FACING
                name.scale = Vector3(0.5, 0.5, 1)
                name.pose.orientation.w = 1.0
                name.lifetime = rospy.Duration(0.5)
                name.id = track.id + 100000
                name.pose.position.x = track.pose.x
                name.pose.position.y = track.pose.y
                name.text =  track.name
                self.ars_array_.markers.append(name)

        self.pub_ars_.publish(self.ars_array_)
        self.ars_array_.markers[:] = []

    def start(self):
        if self.topic_can1:
            rospy.Subscriber('/can_dumper_node/can1_data', RadarArray, self.callback_ssr, queue_size = 1)
        if self.topic_can2:
            rospy.Subscriber('/can_dumper_node/can2_data', RadarArray, self.callback_ssr, queue_size = 1)
        if self.topic_can5:
            rospy.Subscriber('/can_dumper_node/can5_data', RadarArray, self.callback_ars, queue_size = 1)
        rospy.spin()
