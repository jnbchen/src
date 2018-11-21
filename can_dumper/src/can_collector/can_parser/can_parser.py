#!/usr/bin/env python

import cantools, os
import numpy as np

class can_parser(object):
    def __init__(self, dbfile):
        self.db = None
        self.lts = {}
        self.sts = {}
        if os.path.isfile(dbfile):
            self.db = cantools.db.load_file(dbfile, strict=False)

    def get_filter(self, channel):
        filter_list = []
        filter_dict = {}
        if channel == 'can0':
            filter_dict['can_id'] = 0x60B
            filter_dict['can_mask'] = 0xFFF
            filter_dict['extended'] = False
            filter_list.append(filter_dict)
            filter_dict['can_id'] = 0x60D
            filter_list.append(filter_dict)
        elif channel == 'can1':
            filter_dict['can_id'] = 0x60C
            filter_dict['can_mask'] = 0xFFF
            filter_dict['extended'] = False
            filter_list.append(filter_dict)
            filter_dict['can_id'] = 0x61C
            filter_list.append(filter_dict)
        elif channel == 'can2':
            filter_dict['can_id'] = 0x62C
            filter_dict['can_mask'] = 0xFFF
            filter_dict['extended'] = False
            filter_list.append(filter_dict)
            filter_dict['can_id'] = 0x63C
            filter_list.append(filter_dict)

        return filter_list

    def decode(self, frame):
        if self.db is None:
            return None
        else:
            try:
                msg = self.db.get_message_by_frame_id(frame.arbitration_id)
                ret = msg.decode(frame.data)
                ret['channel'] = frame.channel
                ret['id'] = frame.arbitration_id

                if True:
                    if frame.arbitration_id in self.lts:
                        self.sts[frame.arbitration_id].append(frame.timestamp-self.lts[frame.arbitration_id])
                        self.lts[frame.arbitration_id] = frame.timestamp
                    else:
                        self.lts[frame.arbitration_id] = frame.timestamp
                        self.sts[frame.arbitration_id] = []

                return ret
            #except (KeyError, ValueError):
            except Exception as e:
                return None

    def report(self):
        for k, v in self.sts.items():
            try:
                msg = self.db.get_message_by_frame_id(k)
                if msg.send_type is not None and int(msg.send_type) > 0:
                    print("%24s(0x%x): cycle_time %d, send_type %s" %(msg.name, k, msg.cycle_time, msg.send_type))
                ar = np.array(v)
                print("0x%x: mean %16f, var %16f, std %16f, max %16f, min %16f" %(k, ar.mean(), ar.var(), ar.std(), ar.max(), ar.min()))
            except Exception as e:
                pass
        self.sts.clear()
        self.lts.clear()
