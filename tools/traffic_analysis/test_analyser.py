#!/usr/bin/env python3

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

import subprocess


def test_calculated_output():
    expected_results = {
        'data msg count': '606',
        'data msg throughput (msgs/s)': '760.9',
        'data payload throughput (bytes/s)': '355037.1',
        'dropped packets': '1086',
        'frame count': '1486',
        'VLAN frame count': '468',
        'throughput (bytes/s)': '592190.9',
        'total frames size (bytes)': '435436',
        'average frame size (bytes)': '293.0',
        'standard deviation of frame sizes (bytes)': '293.3',
        'sniffing time (s)': '0.7',
        'average interval (s)': '0.000495149',
        'jitter (stdev) of intervals (s)': '0.030224991'
    }
    output = subprocess.run(['python3', 'traffic_analysis.py', '-p', 'captures/example-vlan-rtps.pcapng', '-f', 'rtps'],
                            check=True, stdout=subprocess.PIPE).stdout.splitlines()

    # NOTE: ignore the horrible complexity of those loops, value the readability... ;-)
    for name, value in expected_results.items():
        for line in output:
            columns = line.decode('utf-8').strip().split('|') # <plugin name>, name, value
            if len(columns) != 3:
                continue # line has no data that we check
            if columns[1].strip() != name:
                continue # line has different name
            output_value = columns[2].strip()
            assert output_value == value, 'wrong value=%s for (name="%s", value=%s)' % (output_value, name, value)
            break # matched name and value
        else: # no break encountered, no match found
            assert False, 'missing name="%s" with value=%s' % (name, value)
