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

"""
Merge timestamps from 2 different source packets
and calculate the difference between the two timestamps.
"""

import csv
import argparse
from decimal import Decimal


parser = argparse.ArgumentParser(description='merge timestamps from 2 CSV source files and output the result to a new CSV')
parser.add_argument('input_send', help='input CSV file of sending side')
parser.add_argument('input_recv', help='input CSV file of receiving side')
parser.add_argument('output_file', help='output CSV file')
args = parser.parse_args()

input_send_file = open(args.input_send, newline='')
send_reader = csv.reader(input_send_file, delimiter=';', quotechar='|')
send_reader = list(send_reader)

input_recv_file = open(args.input_recv, newline='')
recv_reader = csv.reader(input_recv_file, delimiter=';', quotechar='|')
recv_reader = list(recv_reader)

send_csv_row_index = 0
recv_csv_row_index = 0

with open(args.output_file, 'w', newline='') as csvfile:
    merged_file = csv.writer(csvfile, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    while send_csv_row_index < len(send_reader) and recv_csv_row_index < len(recv_reader):
        send_rtps_sequence_id = int(send_reader[send_csv_row_index][0])
        recv_rtps_sequence_id = int(recv_reader[recv_csv_row_index][0])

        # gaps?
        if recv_rtps_sequence_id > send_rtps_sequence_id:
            send_csv_row_index += 1
        if send_rtps_sequence_id > recv_rtps_sequence_id:
            recv_csv_row_index += 1

        # do work and go to next
        if recv_rtps_sequence_id == send_rtps_sequence_id:
            ts_tx = Decimal(send_reader[send_csv_row_index][1])
            ts_rx = Decimal(recv_reader[recv_csv_row_index][1])
            ts_diff = ts_rx - ts_tx
            merged_file.writerow([recv_rtps_sequence_id, ts_tx, ts_rx, ts_diff])
            send_csv_row_index += 1
            recv_csv_row_index += 1