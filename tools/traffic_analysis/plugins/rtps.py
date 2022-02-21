# Copyright 2022 NXP. All rights reserved.
# Disclaimer
# 1. The NXP Software/Source Code is provided to Licensee "AS IS" without any
# warranties of any kind. NXP makes no warranties to Licensee and shall not
# indemnify Licensee or hold it harmless or any reason related to the NXP
# Software/Source Code or otherwise be liable to the NXP customer. The NXP
# customer acknowledges and agrees that the NXP Software/Source Code is
# provided AS-IS and accepts all risks of utilizing the NXP Software under the
# conditions set forth according to this disclaimer.
# *
# 2. NXP EXPRESSLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE, AND NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS. NXP
# SHALL HAVE NO LIABILITY TO THE NXP CUSTOMER, OR ITS SUBSIDIARIES, AFFILIATES,
# OR ANY OTHER THIRD PARTY FOR ANY DAMAGES, INCLUDING WITHOUT LIMITATION,
# DAMAGES RESULTING OR ALLEGED TO HAVE RESULTED FROM ANY DEFECT, ERROR OR
# OMISSION IN THE NXP SOFTWARE/SOURCE CODE, THIRD PARTY APPLICATION SOFTWARE
# AND/OR DOCUMENTATION, OR AS A RESULT OF ANY INFRINGEMENT OF ANY INTELLECTUAL
# PROPERTY RIGHT OF ANY THIRD PARTY. IN NO EVENT SHALL NXP
# BE LIABLE FOR ANY INCIDENTAL, INDIRECT, SPECIAL, EXEMPLARY, PUNITIVE, OR
# CONSEQUENTIAL DAMAGES (INCLUDING LOST PROFITS) SUFFERED BY NXP CUSTOMER OR
# ITS SUBSIDIARIES, AFFILIATES, OR ANY OTHER THIRD PARTY ARISING OUT OF OR
# RELATED TO THE NXP SOFTWARE/SOURCE CODE EVEN IF NXP HAS BEEN ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGES.

"RTPS dissector plugin for analyzing DDS traffic"


import os
import plugin
import logging as log
import statistics as stat
from datetime import datetime


class RtpsFramePlugin(plugin.AnalyzerPlugin):
    def __init__(self):
        super().__init__()
        self._name = 'rtps.frame'
        self.total_size = self.pkt_count = self.prev_timestamp = self.first_timestamp = self.vlan_pkt_count = 0
        self.sizes = []

    def processPacket(self, p):
        if not hasattr(p, 'rtps'):
            return
        self.total_size += int(p.length)
        self.sizes += [int(p.length)]
        cur_timestamp = float(p.frame_info.time_epoch)
        if hasattr(p, 'vlan'):
            self.vlan_pkt_count += 1
        self.pkt_count += 1
        if self.pkt_count == 1:
            self.first_timestamp = cur_timestamp
        self.prev_timestamp = cur_timestamp

    def reportStatistics(self):
        if self.pkt_count == 0:
            return None

        sniff_time = self.prev_timestamp - self.first_timestamp
        stats = {
            'frame count':                          str(self.pkt_count),
            'VLAN frame count':                     '{}'.format(self.vlan_pkt_count),
            'VLAN frame %':                         '{:.1%}'.format(float(self.vlan_pkt_count)/float(self.pkt_count)),
            'total frames size (bytes)':            '{}'.format(self.total_size),
            'average frame size (bytes)':           '{:.1f}'.format(stat.mean(self.sizes))
        }
        if len(self.sizes) <= 1:
            stats['standard deviation of frame sizes (bytes)'] = 'Not calculated, {} frame captured in total'.format(len(self.sizes))
        else:
            stats['standard deviation of frame sizes (bytes)'] = '{:.1f}'.format(stat.stdev(self.sizes))
        stats['sniffing time (s)'] = '{:.1f}'.format(sniff_time)
        if sniff_time == 0:
            stats['throughput (bytes/s)'] = 'Not calculated due to {} s sniffing time'.format(sniff_time)
        else:
            stats['throughput (bytes/s)'] = '{:.1f}'.format(self.total_size/sniff_time)
        return stats


class RtpsIntervalsPlugin(plugin.AnalyzerPlugin):
    def __init__(self):
        super().__init__()
        self._name = 'rtps.intervals'
        self.intervals = None

    def processPacket(self, p):
        if not hasattr(p, 'rtps'):
            return
        cur_timestamp = float(p.frame_info.time_epoch)
        if self.intervals == None:
            self.intervals = []
        else:
            self.intervals += [cur_timestamp - self.prev_timestamp]
        self.prev_timestamp = cur_timestamp

    def reportStatistics(self):
        if self.intervals == None:
            return None
        if len(self.intervals) <= 1:
            return {
                'average and jitter (s)':                   'Not calculated, {} interval captured in total'.format(len(self.intervals))
            }
        else:
            return {
                'average interval (s)':                     '{:.9f}'.format(stat.mean(self.intervals)),
                'jitter (stdev) of intervals (s)':          '{:.9f}'.format(stat.stdev(self.intervals))
            }

RTPS_SUBMSG_INFO_TS     = 0x09
RTPS_SUBMSG_DATA        = 0x15

class RtpsDataPlugin(plugin.AnalyzerPlugin):
    def __init__(self):
        super().__init__()
        self._name = 'rtps.data'
        self._filters = [
            ('udp_dstport', 'specify UDP destination ports which are used for DATA submessages'),
            ('writer_entity_key', 'specify writer entity key in a DATA submessage')
        ]
        self.data_frame_count = 0
        self.total_size = 0
        self.dropped_data_packets = 0
        self.resent_packets = 0
        self.rtps_intervals = []
        self.rtps2sniff_intervals = []
        self.capture_timestamps = []
        self.sequence_numbers = []
        self.prev_rtps_timestamp = None

    def processFilter(self, p, filter_name, filter_value):
        if filter_name == self.name + '.udp_dstport':
            if hasattr(p, 'udp') and (p.udp.dstport == filter_value):
                return True
            return False
        elif filter_name == self.name + '.writer_entity_key':
            if not hasattr(p, 'rtps') or not hasattr(p.rtps, 'sm') \
            or not isinstance(p.rtps.sm.id, list):
                return False
            for id, tree in zip(p.rtps.sm.id, p.rtps.sm.id_tree):
                if int(id, base=16) != RTPS_SUBMSG_DATA:
                    continue
                if int(tree.wrEntityId_tree.entityKey, 16) == int(filter_value, 16):
                    return True
            return False
        else:
            return True

    def processPacket(self, p):
        if not hasattr(p, 'rtps'):
            return
        # skip if there are no RTPS submessages
        if not hasattr(p.rtps, 'sm'):
            return

        # check if id is really a list / is iteratable
        if not isinstance(p.rtps.sm.id, list):
            return

        cur_rtps_timestamp = None
        for id, tree in zip(p.rtps.sm.id, p.rtps.sm.id_tree):
            # only handle INFO_TS sub-messages (= 0x09) which contains the RTPS timestamp
            if int(id, base=16) == RTPS_SUBMSG_INFO_TS:
                cur_rtps_timestamp = datetime.strptime(tree.timestamp[:-8], '%b %d, %Y %H:%M:%S.%f').timestamp()
            # skip RTPS without DATA sub-messages (= 0x15)
            if int(id, base=16) != RTPS_SUBMSG_DATA:
                continue
            cur_sequence_num = int(tree.seqNumber)
            # skip RTPS discovery packets
            if cur_sequence_num == 1:
                continue

            self.total_size += int(tree.octetsToNextHeader)
            # frame timestamp
            cur_timestamp = float(p.frame_info.time_epoch)
            timestamp_shift = os.getenv('RTPS_DATE_TIMESTAMP_SHIFT')
            if timestamp_shift != None:
                cur_timestamp += float(timestamp_shift)

            self.data_frame_count += 1
            if self.data_frame_count == 1:
                self.first_timestamp = cur_timestamp
            self.prev_timestamp = cur_timestamp

            if len(self.sequence_numbers) > 0:
                delta_sequence_nums = cur_sequence_num - self.sequence_numbers[-1]
                if delta_sequence_nums <= 0:
                    log.warn('last sequence number: %d, cur_sequence_num: %d', self.sequence_numbers[-1], cur_sequence_num)
                    self.resent_packets += 1
                    continue
                else:
                    self.dropped_data_packets += delta_sequence_nums - 1
                    if delta_sequence_nums > 1:
                        # reset rtps timestamp to not calculate interval when packets are dropped
                        self.prev_rtps_timestamp = None
            self.sequence_numbers.append(cur_sequence_num)

            if self.prev_rtps_timestamp != None:
                self.rtps_intervals += [cur_rtps_timestamp - self.prev_rtps_timestamp]
            else:
                self.rtps_intervals += [0.0]
            self.prev_rtps_timestamp = cur_rtps_timestamp
            # interval between the RTPS production and sniffed moment
            if cur_rtps_timestamp != None:
                self.rtps2sniff_intervals += [cur_timestamp - cur_rtps_timestamp]
            else:
                log.warn('Data submessage does not have a timestamp')

            self.capture_timestamps.append(cur_timestamp)

    def reportStatistics(self):
        if self.data_frame_count == 0:
            return None
        sniff_time = self.prev_timestamp - self.first_timestamp
        stats = {
            'data msg count':                        str(self.data_frame_count),
            'data msg throughput (msgs/s)':          '{:.1f}'.format(self.data_frame_count/sniff_time),
            'data payload throughput (bytes/s)':     '{:.1f}'.format(self.total_size/sniff_time),
            'dropped packets':                       str(self.dropped_data_packets),
            'dropped packets %':                     '{:.2%}'.format(self.dropped_data_packets/self.data_frame_count),
            're-sent packets':                       str(self.resent_packets),
        }
        if self.rtps_intervals == []:
            stats['production interval: average time & jitter (stdev) (s)'] = 'No published data captured'
        else:
            stats['production interval: average time (s)']      = '{:.9f}'.format(stat.mean(self.rtps_intervals))
            stats['production interval: jitter (stdev) (s)']    = '{:.9f}'.format(stat.stdev(self.rtps_intervals))
            stats['production interval: min time (s)']          = '{:.9f}'.format(min(self.rtps_intervals))
            stats['production interval: max time (s)']          = '{:.9f}'.format(max(self.rtps_intervals))
            stats['production interval: max-min diff (s)']      = '{:.9f}'.format(max(self.rtps_intervals) - min(self.rtps_intervals))
        if self.rtps2sniff_intervals == []:
            stats['packet transmission: average time & jitter (stdev) (s)'] = 'No published data captured'
        else:
            stats['packet transmission: average time (s)']   = '{:.9f}'.format(stat.mean(self.rtps2sniff_intervals))
            stats['packet transmission: jitter (stdev) (s)'] = '{:.9f}'.format(stat.stdev(self.rtps2sniff_intervals))
            stats['packet transmission: min time (s)']       = '{:.9f}'.format(min(self.rtps2sniff_intervals))
            stats['packet transmission: max time (s)']       = '{:.9f}'.format(max(self.rtps2sniff_intervals))
            stats['packet transmission: max-min diff (s)']   = '{:.9f}'.format(max(self.rtps2sniff_intervals) - min(self.rtps2sniff_intervals))
        return stats

    def reportCsvOutput(self):
        return [self.sequence_numbers, self.capture_timestamps, self.rtps2sniff_intervals, [0.0] + self.rtps_intervals]
