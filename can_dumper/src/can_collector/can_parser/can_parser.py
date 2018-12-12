#!/usr/bin/env python

import cantools, os
import numpy as np

class can_parser(object):
    def __init__(self, dbfile):
        self.db = None
        if os.path.isfile(dbfile):
            self.db = cantools.db.load_file(dbfile, strict=False)

    def get_filter(self, channel):
        filter_list = []
        if channel == 'can5':
            filter_list = [{'can_id':0x60B,'can_mask':0xFFF,'extended':False}, \
                           {'can_id':0x60C,'can_mask':0xFFF,'extended':False}, \
                           {'can_id':0x60D,'can_mask':0xFFF,'extended':False}]
        elif channel == 'can1':
            filter_list = [{'can_id':0x60C,'can_mask':0xFFF,'extended':False}, \
                           {'can_id':0x61C,'can_mask':0xFFF,'extended':False}]
        elif channel == 'can2':
            filter_list = [{'can_id':0x62C,'can_mask':0xFFF,'extended':False}, \
                           {'can_id':0x63C,'can_mask':0xFFF,'extended':False}]
        elif channel == 'can0':
            filter_list = [{'can_id':0x784,'can_mask':0xFFF,'extended':False}, \
                           {'can_id':0x774,'can_mask':0xFFF,'extended':False}, \
                           {'can_id':0x772,'can_mask':0xFFF,'extended':False}, \
                           {'can_id':0x771,'can_mask':0xFFF,'extended':False}]
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

                return ret
            #except (KeyError, ValueError):
            except Exception as e:
                return None