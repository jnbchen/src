#!/usr/bin/env python

import can, json, time, sched, can_parser, os
import rospy
from can_dumper.msg import *

class can_collector(object):
    def __init__(self):
        self.bus = []
        self.notifier = []
        self.listener = {}
        self.parser = {}
        self.radar_array_1 = RadarArray()
        self.radar_array_2 = RadarArray()
        self.radar_array_5 = RadarArray()
        self.radar_ = Radar()
        self.ultra_sonic_ = UltraSonic()
        self.routelist = []
        self.buffer_ = {}
        self.routeid = None
        self.topic_ = True
        self.list_ssr_ = [0x60C, 0x61C, 0x62C, 0x63C]
        self.list_ars_ = [0x60B, 0x60C, 0x60D]
        self.list_ultra_ = [0x771, 0x772, 0x774, 0x784]
        self.pub_0 = rospy.Publisher('/can_dumper_node/can0_data', UltraSonic, queue_size=1000)
        self.pub_1 = rospy.Publisher('/can_dumper_node/can1_data', RadarArray, queue_size=1000)
        self.pub_2 = rospy.Publisher('/can_dumper_node/can2_data', RadarArray, queue_size=1000)
        self.pub_5 = rospy.Publisher('/can_dumper_node/can5_data', RadarArray, queue_size=1000)

        config             = rospy.get_param('~device_config_file', './')
        self.interval_     = rospy.get_param('~collect_interval', 10)
        self.broadcast_    = rospy.get_param('~broadcast_time', 50)

        with open(config, 'r') as f:
            record = json.load(f)
            if 'db_path' in record:
                db_dir = record['db_path'] + '/'
            else:
                db_dir = "./"
            if 'device' in record:
                for conf in record['device']:
                    channel = conf['channel']
                    self.parser[channel] = []
                    bus_filter = []
                    for fdb in conf['db']:
                        d = can_parser.can_parser(db_dir+fdb)
                        print(db_dir+fdb)
                        self.parser[channel].append(d)
                        filter_parser = d.get_filter(channel)
                        bus_filter += filter_parser

                    b = can.interface.Bus(bustype=conf['bustype'], channel=conf['channel'], bitrate=conf['bitrate'], can_filters=bus_filter)
                    self.bus.append(b)
                    l = can.BufferedReader()
                    n = can.Notifier(b, [l])
                    self.notifier.append(n)
                    self.listener[channel] = l
            if 'route_id' in record:
                self.routeid = record['route_id']

        self.job = sched.scheduler(time.time, time.sleep)

    def collect(self, args=None):
        self.job.enter(self.interval_/1000.0, 1, self.collect, argument=())
        for k, l in self.listener.items():
            qsize = l.buffer.qsize()
            for q in range(qsize):
                m = l.get_message(0)
                if m is not None:
                    for p in self.parser[k]:
                        r = p.decode(m)
                        if r is not None:
                           # if r['channel']=='can5':
                           #     print(r)
                            key = r['channel'] + '/' + str(r['id'])
                            self.buffer_[key] = r
                            break

    def build_msg(self):
        buffer = {}
        speicher = {}
        iter = 1
        for i in self.buffer_:
            r = self.buffer_[i]
            radar = Radar()
            if (r['channel'] == 'can1' or r['channel'] == 'can2') and r['id'] in self.list_ssr_:
                device = str((r['id']&0x0F0)>>4)
                radar.id = r['Track_ID']
                radar.radar_type = 'ContiSSR208_' + device
                #print(radar.radar_type)
                radar.is_object = False
                radar.classification = 0xFF
                radar.distance_x = r['Track_LatDispl']
                radar.distance_y = r['Track_LongDispl']
                radar.relative_velocity_x = r['Track_VrelLat']
                radar.relative_velocity_y = r['Track_VrelLong']
                radar.roll_count = r['TrackSt_RollCount']
                if r['id'] == 0x60C or r['id'] == 0x61C:
                    self.radar_array_1.header.stamp = rospy.get_rostime()
                    self.radar_array_1.header.frame_id = 'radar'
                    self.radar_array_1.radar_array.append(radar)
                elif r['id'] == 0x62C or r['id'] == 0x63C:
                    self.radar_array_2.header.stamp = rospy.get_rostime()
                    self.radar_array_2.header.frame_id = 'radar'
                    self.radar_array_2.radar_array.append(radar)

            elif r['id'] in self.list_ars_ and r['channel'] == 'can5':
                id = r['Obj_ID']
                if id not in buffer.keys():
                    buffer[id] = r
                else:
                    buffer[id].update(r)
                    radar.is_object = False
                    #print(len(buffer[id].keys()))
                    #print(buffer[id].keys())
                if len(buffer[id].keys()) == 24:
                    res = buffer[id]
                    radar.id = res['Obj_ID']
                    radar.radar_type = 'ContiARS408'
                    radar.is_object = True
                    radar.rcs_value = res['Obj_RCS']
                    radar.distance_x = res['Obj_DistLat']
                    radar.distance_y = res['Obj_DistLong']
                    radar.relative_velocity_x = res['Obj_VrelLat']
                    radar.relative_velocity_y = res['Obj_VrelLong']
                    radar.probolity_of_exist = res['Obj_ProbOfExist']
                    radar.classification = res['Obj_Class']
                    radar.measure_state = res['Obj_MeasState']
                    radar.object_size_x = res['Obj_Length']
                    radar.object_size_y = res['Obj_Width']
                    radar.angle = res['Obj_OrientationAngle']
                    radar.name = self.switch(radar.classification).get('name')
                    radar.r = self.switch(radar.classification).get('r')
                    radar.g = self.switch(radar.classification).get('g')
                    radar.b = self.switch(radar.classification).get('b')
                    radar.a = self.switch(radar.classification).get('a')
                    #print(radar)

                    self.radar_array_5.header.stamp = rospy.get_rostime()
                    self.radar_array_5.header.frame_id = 'radar'
                    self.radar_array_5.radar_array.append(radar)

            elif r['id'] in self.list_ultra_ and r['channel'] == 'can0':
                if iter not in speicher.keys():
                    speicher[iter] = r
                    #print(iter)
                else:
                    speicher[iter].update(r)
                    #print(speicher)
                    #print(iter)
                if len(speicher[iter].keys()) == 18:
                    #print('got')
                    result = speicher[iter]
                    self.ultra_sonic_.corner_A_x = 0.01*result['Parking_CornerA_X1']
                    self.ultra_sonic_.corner_A_y = 0.01*result['Parking_CornerA_Y1']
                    self.ultra_sonic_.corner_B_x = 0.01*result['Parking_CornerB_X2']
                    self.ultra_sonic_.corner_B_y = 0.01*result['Parking_CornerB_Y2']
                    self.ultra_sonic_.corner_C_x = 0.01*result['Parking_CornerC_X3']
                    self.ultra_sonic_.corner_C_y = 0.01*result['Parking_CornerC_Y3']
                    self.ultra_sonic_.corner_D_x = 0.01*result['Parking_CornerD_X4']
                    self.ultra_sonic_.corner_D_y = 0.01*result['Parking_CornerD_Y4']
                    self.ultra_sonic_.lenth_l = 0.01*result['Parking_left_l_Distance']
                    self.ultra_sonic_.deepth_l = 0.01*result['Parking_left_d_Distance']
                    self.ultra_sonic_.lenth_r = 0.01*result['Parking_right_l_Distance']
                    self.ultra_sonic_.deepth_r = 0.01*result['Parking_right_d_Distance']
                    iter += 1
                    #print(iter)

    def switch(self, var):
        return {
                self.radar_.POINT: {'name':'POINTS','r':1.0,'g':1.0,'b':1.0,'a':1.0},
                self.radar_.CAR: {'name':'CAR','r':0.0,'g':0.0,'b':1.0,'a':1.0},
                self.radar_.TRUCK: {'name':'TRUCK','r':1.0,'g':0.0,'b':1.0,'a':1.0},
                self.radar_.PEDESTRIAN: {'name':'PEDESTRIAN','r':1.0,'g':0.0,'b':0.0,'a':1.0},
                self.radar_.MOTORCYCLE: {'name':'MOTORCYCLE','r':0.0,'g':1.0,'b':1.0,'a':1.0},
                self.radar_.BICYCLE: {'name':'BICYCLE','r':1.0,'g':1.0,'b':0.0,'a':1.0},
                self.radar_.WIDE: {'name':'WIDE','r':1.0,'g':0.0,'b':1.0,'a':1.0}
        }.get(var,{'name':'UNKNOWN','r':1.0,'g':1.0,'b':1.0,'a':1.0})

    def broadcast(self, args=None):
        self.build_msg()
        #print(self.radar_array_)
        if self.topic_:
            self.pub_0.publish(self.ultra_sonic_)
            self.pub_1.publish(self.radar_array_1)
            self.pub_2.publish(self.radar_array_2)
            self.pub_5.publish(self.radar_array_5)
            self.buffer_ = {}
            self.radar_array_1.radar_array[:] = []
            self.radar_array_2.radar_array[:] = []
            self.radar_array_5.radar_array[:] = []
        self.job.enter(self.broadcast_/1000.0, 3, self.broadcast, argument=())

    def diagnostic(self, bus_id):
        if bus_id < 0 or bus_id >= len(self.bus):
            return -1

        curr = time.time()
        # hard-corded the name as can0-canxx, TBD
        k = "can" + str(bus_id)
        for p in self.parser[k]:
            if not p.lts.items():
                return 1
            # for the can message contain in dbc but have not received, the
            # last timestamp will not appear in the lts dictionary
            for mid, last in p.lts.items():
                # 3 seconds without any message with can id == mid received
                if (curr-last) > 3.0:
                    return mid
        return 0

    def start(self):
        self.job.enter(self.interval_/1000.0, 1, self.collect, argument=())
        self.job.enter(self.broadcast_/1000.0, 3, self.broadcast, argument=())
        self.job.run()
