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
