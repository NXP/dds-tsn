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