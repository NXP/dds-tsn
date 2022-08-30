# Traffic Analyser framework

This directory contains a framework to analyse `.pcapng` files, create statistics for a specific protocol and export certain protocol properties to a `.csv` file. It contains a plugin system for protocols which makes it easy for you to add your favourite protocol.

Our main focus during the development of this framework was the RTPS protocol.

# Directory structure

- `traffic_analysis` contains scripts to analyze network traffic in terms of throughput, latency, packet error rate, etc.
- `traffic_analysis/captures` contains example wireshark capture files for testing.

# Requirements
```bash
pip3 install pyshark tabulate pytest
apt install tshark
```

# Features
```bash
cd dds_tsn_linux_host/traffic_analysis
python3 traffic_analysis.py -h
usage: traffic_analysis.py [-h] [-i NETWORK_INTERFACE] [-d DURATION]
                           [-p FILE_PATH] [-f FILTER] [-v]
Sniff network traffic or read from a file and compute various statistics
optional arguments:
  -h, --help            show this help message and exit
  -i NETWORK_INTERFACE, --interface NETWORK_INTERFACE
                        network interface to open for packet sniffing
  -d DURATION, --duration DURATION
                        duration in seconds of live packet sniffing on the
                        specified network interface
  -p FILE_PATH, --captureFilePath FILE_PATH
                        PCAP capture file path for packet processing
  -f FILTER, --wiresharkFilter FILTER
                        Wireshark display filter to focus processing on
                        specific packets
  -x name value, --pluginFilters name value
                        display filters on plugin level
  -s, --show-pluginFilters
                        show available display filters on plugin level
  -v, --verbose         verbose mode with extra logging
```

# Sniff wireshark with HW timestamps
Wireshark nowadays supports (under Linux) to use HW timestamps on capable Ethernet interfaces.
This can give you more accurate results for transport time calculations, etc.

To check if your hardware support HW TX and RX timestamps, run:
```
ethtool -T <interface-name>
```
You should see that timestamping ```ALL``` packets should be supported on TX and RX:
```
Hardware Receive Filter Modes:
        none                  (HWTSTAMP_FILTER_NONE)
        ...
        all                   (HWTSTAMP_FILTER_ALL)
```

Run a capability check, if also tshark can use the HW timestamping:
```
sudo tshark -i <interface-name> --list-time-stamp-types
```
This should print out at least ```adapter_synced``` or ```adapter_unsynced```.

Then, you can sniff traffic with tshark and save the results to a `.pcapng` file:
```
sudo tshark -i <interface-name> --time-stamp-type adapter_unsynced -w /tmp/output-file.pcapng
```

Note: tshark will always try to enable HW timestamps, but it doesn't display any error on failure.

## Example output
```
Statistics with a global pyshark filter: rtps
 plugin         | name                                      | value
----------------+-------------------------------------------+------------------
 ethernet       | frame count                               | 1486
                | VLAN frame count                          | 468
                | VLAN frame %                              | 31.5%
                | total frames size (bytes)                 | 435436
                | sniffing time (s)                         | 0.7
                | average frame size (bytes)                | 293.0
                | standard deviation of frame sizes (bytes) | 293.3
                | throughput (bytes/s)                      | 592190.9
 rtps.data      | data msg count                            | 606
                | data msg throughput (msgs/s)              | 760.9
                | data payload throughput (bytes/s)         | 355037.1
                | dropped packets                           | 1086
                | dropped packets %                         | 179.21%
                | re-sent packets                           | 530
                | production interval: average time (s)     | 7542.373490001
                | production interval: jitter (stdev) (s)   | 65752.761288732
                | production interval: min time (s)         | 0.000000000
                | production interval: max time (s)         | 573219.297930002
                | production interval: max-min diff (s)     | 573219.297930002
                | packet transmission: average time (s)     | 15086.234305899
                | packet transmission: jitter (stdev) (s)   | 92368.043753895
                | packet transmission: min time (s)         | -0.090008259
                | packet transmission: max time (s)         | 573230.671374559
                | packet transmission: max-min diff (s)     | 573230.761382818
 rtps.frame     | frame count                               | 1486
                | VLAN frame count                          | 468
                | VLAN frame %                              | 31.5%
                | total frames size (bytes)                 | 435436
                | average frame size (bytes)                | 293.0
                | standard deviation of frame sizes (bytes) | 293.3
                | sniffing time (s)                         | 0.7
                | throughput (bytes/s)                      | 592190.9
 rtps.intervals | average interval (s)                      | 0.000495149
                | jitter (stdev) of intervals (s)           | 0.030224991
```

## Running integration tests
We have at least one integration test (see [here](test_analyser.py)) which checks all calculated values on a recorded sniff. To test it, run:
```bash
pytest-3
```

## Known limitations
If you trigger the following exception in `pyshark` module `layers.py`, you can patch it using [this post](https://github.com/nikirill/pyshark/commit/fc49ca9405f5c770fce15e77841be45f9e6b4dee):

```python
Traceback (most recent call last):
...
  File "/home/yuting/.local/lib/python3.8/site-packages/pyshark/packet/layer.py", line 34, in __getattr__
    val = self.get_field(item)
  File "/home/yuting/.local/lib/python3.8/site-packages/pyshark/packet/layer.py", line 254, in get_field
    self._convert_showname_field_names_to_field_names()
  File "/home/yuting/.local/lib/python3.8/site-packages/pyshark/packet/layer.py", line 275, in _convert_showname_field_names_to_field_names
    for field_name in self._all_fields:
RuntimeError: dictionary keys changed during iteration
```
