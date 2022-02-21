# RTPS CSV merging tool

`rtps_csv_merge.py` computes transmission latency of RTPS packets from a sending machine to a receiving machine. The two CSV files with RTPS packet information from the sending and receiving machines have to be first computed with `traffic_analysis.py` from the `tshark` sniffs. Then, `rtps_csv_merge.py` reads the CSV files and computes timestamp differences of RTPS packets with the same sequence ID from the two input CSV files.

The source packets come from:
* `tshark` sniff on the sending side (extracted with `traffic_analysis.py` CSV export)
* `tshark` sniff on tne receiving side (extracted with `traffic_analysis.py` CSV export)

The time difference calculation: `recv-send-difference = recv timestam - send timestamp`

It will merge two timestamps based on the sequence ID of the sample packet and skip IDs which aren't available in both source packets.

Usage:
```
$ python3 rtps_csv_merge.py -h
usage: rtps_csv_merge.py [-h] input_send input_recv output_file

merges 3 CSVs with different timestamp sources

positional arguments:
  input_send   input CSV file of sending side
  input_recv   input CSV file of receiving side
  output_file  output CSV file

optional arguments:
  -h, --help   show this help message and exit
```

The resulting CSV will have the following content:
```
<RTPS/DDS-sequence-ID>;<send-timestamp>;<recv-timestamp>;<recv-send-difference>
```

## Example
To sniff the network interface, generate the `.pcapng` file and the `.csv` file in one go, follow instructions [here](../traffic_analysis/README.md).

Assume you already have two `.pcapng` files available in the `traffic_analysis` directory captured from the ROS2 publisher and the subscriber respectively.
1. First generate the `.csv` file for each `.pcapng` file
   ```bash
   cd dds-tsn/tools/traffic_analysis
   python3 traffic_analysis.py -p publisher.pcapng -c rtps.data ../rtps_csv_merge/publisher.csv -f 'udp.srcport == 46278'
   python3 traffic_analysis.py -p subscriber.pcapng -c rtps.data ../rtps_csv_merge/subscriber.csv -f 'udp.srcport == 46278'
   ```
1. Merge the timestamps and generate the output CSV file:
   ```bash
   cd dds-tsn/tools/rtps_csv_merge
   python3 rtps_csv_merge.py publisher.csv subscriber.csv merged.csv
   ```