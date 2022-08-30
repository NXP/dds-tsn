# Copyright 2022 NXP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"Generic Ethernet frame dissector plugin"


import plugin
import statistics as stat
from datetime import datetime


class EthernetFramePlugin(plugin.AnalyzerPlugin):
    def __init__(self):
        super().__init__()
        self._name = 'ethernet'
        self.total_size = self.pkt_count = self.prev_timestamp = self.first_timestamp = self.vlan_pkt_count = 0
        self.sizes = []

    def processPacket(self, p):
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
        stats =  {
            'frame count':                          str(self.pkt_count),
            'VLAN frame count':                     '{}'.format(self.vlan_pkt_count),
            'VLAN frame %':                         '{:.1%}'.format(float(self.vlan_pkt_count)/float(self.pkt_count)),
            'total frames size (bytes)':            '{}'.format(self.total_size),
            'sniffing time (s)':                    '{:.1f}'.format(sniff_time),
        }
        if len(self.sizes) <= 1:
            stats['average and deviation of frame sizes (bytes)'] = 'Not calculated, {} frame captured in total'.format(len(self.sizes))
        else:
            stats['average frame size (bytes)'] =                '{:.1f}'.format(stat.mean(self.sizes))
            stats['standard deviation of frame sizes (bytes)'] = '{:.1f}'.format(stat.stdev(self.sizes))
        if sniff_time == 0:
            stats['throughput (bytes/s)'] = 'Not calculated due to {} s sniffing time'.format(sniff_time)
        else:
            stats['throughput (bytes/s)'] = '{:.1f}'.format(self.total_size/sniff_time)
        return stats
