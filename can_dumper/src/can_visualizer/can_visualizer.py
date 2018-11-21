#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from can_dumper.msg import *

class can_visualizer(object):
    def __init__(self):
        """ self.marker_ = Marker()
        self.marker_.action = visualization_msgs::Marker::ADD
        self.marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING
        self.marker_.scale.z = 0.1
        self.marker_.pose.orientation.w = 1.0
        self.marker_.lifetime = line_strip_.lifetime = ros::Duration(0.2) """

        self.point_ = Marker()
        self.point_.header.frame_id = 'radar'
        self.point_.header.stamp = rospy.get_rostime()
        self.point_.type = Marker.POINTS
        self.point_.action = Marker.ADD
        self.point_.scale.z = 0.1
        self.point_.pose.orientation.w = 1.0
        self.point_.lifetime = rospy.Duration(0.2)

        self.point_.color.r = 1.0 #self.marker_.color.r = 1.0
        self.point_.color.g = 1.0 #self.marker_.color.r = 1.0
        self.point_.color.b = 1.0 #self.marker_.color.r = 1.0
        self.point_.color.a = 1.0 #self.marker_.color.r = 1.0

        self.marker_array_ = MarkerArray()
        self.pub_ = rospy.Publisher('/can_dumper_node/can_visualization', MarkerArray, queue_size=20)

    def callback(self, msg):
        self.marker_array_.markers[:] = []
        rate = rospy.Rate(30)
        for i in len(msg.radar_array):
            track = msg.radar_array[i]
            self.point_.id = track.id
            self.point_.pose.position.x = track.distance_x
            self.point_.pose.position.y = track.distance_y
            
            """ self.marker_.id = track.id
            self.marker_.pose.position.x = track.distance_x
            self.marker_.pose.position.y = track.distance_y
            text = str(track.track_id)+' v_x:'+str(track.relative_velocity_x)+' v_y:'+str(track.relative_velocity_y)
            self.marker_.text = text """

            #self.marker_array_.markers.append(self.marker_)
            self.marker_array_.markers.append(self.point_)
        
 
        self.pub_.publish(self.marker_array_)
        print('publishing')
        rate.sleep()

    def start(self):
        rospy.Subscriber('/can_dumper_node/can_data', RadarArray, self.callback, queue_size=20)
        rospy.spin()
