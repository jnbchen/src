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
        self.radar_array_ = RadarArray()
        self.radar_array_.header.frame_id = 'radar'
        self.radar_ = Radar()
        self.routelist = []
        self.buffer_ = {}
        self.routeid = None
        self.topic_ = True
        self.pub_ = rospy.Publisher('/can_dumper_node/can_data', RadarArray, queue_size=20)

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
            self.buffer_ = {}
            for q in range(qsize):
                m = l.get_message(0)
                if m is not None:
                    for p in self.parser[k]:
                        r = p.decode(m)
                        if r is not None:
                            self.assign(r)
                            break

                if self.routeid is not None and m.arbitration_id in self.routeid:
                    if m.channel is not None or m.channel != k:
                        m.channel = k
                    self.routelist.append(m)

        # forward speed and steer wheel information to other bus devices
        if len(self.routelist) > 0:
            for m in self.routelist:
                for canbus in self.bus:
                    if canbus.channel != m.channel:
                        try:
                            canbus.send(m, 0.001)
                        except Exception as e:
                            print str(e)
                            pass
            self.routelist[:] = []

    def assign(self, r):
        if (r['channel'] == 'can1' or r['channel'] == 'can2'):
            device = str((r['id']&0x0F0)>>4)
            self.radar_.id = r['Track_ID']
            self.radar_.radar_type = 'ContiSSR208_' + device
            self.radar_.is_object = False
            self.radar_.distance_x = r['Track_LatDispl']
            self.radar_.distance_y = r['Track_LongDispl']
            self.radar_.relative_velocity_x = r['Track_VrelLat']
            self.radar_.relative_velocity_y = r['Track_VrelLong']

            self.radar_array_.radar_array.append(self.radar_)
            # print('SSR')
        
        elif r['channel'] == 'can0':
            id = r['Obj_ID']
            # device = str((id&0x0F0)>>4)
            if id not in self.buffer_:
                self.buffer_[id] = r
            elif self.buffer_[id].keys != r.keys():
                self.buffer_[id].update(r)
                self.radar_.id = self.buffer_[id]['Obj_ID']
                self.radar_.radar_type = 'ContiARS408'
                self.radar_.is_object = True
                self.radar_.distance_x = self.buffer_[id]['Obj_DistLat']
                self.radar_.distance_y = self.buffer_[id]['Obj_DistLong']
                self.radar_.relative_velocity_x = self.buffer_[id]['Obj_VrelLat']
                self.radar_.relative_velocity_y = self.buffer_[id]['Obj_VrelLong']
                self.radar_.classification = self.buffer_[id]['Obj_Class']
                self.radar_.object_size.x = self.buffer_[id]['Obj_Width']
                self.radar_.object_size.y = self.buffer_[id]['Obj_Length']
                self.radar_.angle = self.buffer_[id]['Obj_OrientationAngle']
            
                self.radar_array_.radar_array.append(self.radar_)
                
    def broadcast(self, args=None):
        if self.topic_:
            self.radar_array_.header.stamp = rospy.get_rostime()
            self.radar_array_.header.frame_id = 'radar'
            self.pub_.publish(self.radar_array_)
            
        self.radar_array_.radar_array[:] = []
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
