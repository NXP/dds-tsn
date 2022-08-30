#!/usr/bin/env python3

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
