# DDS-TSN integration demo
This repository demonstrates basic advantages of integrating the [Data Distribution Service (DDS)](https://en.wikipedia.org/wiki/Data_Distribution_Service) and [Time-Sensitive Networking (TSN) Ethernet](https://en.wikipedia.org/wiki/Time-Sensitive_Networking). The demo is based on the [Gazebo plugin `gazebo_ros_diff_drive`](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros#TestingGazeboandROS2integration), modeling a differential drive vehicle in the [Robot Operating System (ROS) 2](https://www.ros.org/) environment, as well as on the GNU/Linux [VLAN](https://tsn.readthedocs.io/vlan.html) and [traffic control](https://tldp.org/HOWTO/Traffic-Control-HOWTO/intro.html) framework.

The structure of this repository is as follows:
- `dds_tsn_demo`: the ROS2 application implementation for the demo;
- `scripts`: script to bring up VLAN interface with QoS configuration on Linux;
- `tools`: a traffic analysis framework and tools to analyze `.pcapng` files for a specific protocol;
- `images`: system block diagram of the demo.
- `licenses`: license files

The demo video below shows the challenging [moose test](https://en.wikipedia.org/wiki/Moose_test) in the Gazebo simulator, where the white ego vehicle performs a time-critical evasive maneuver. Three different scenarios are shown:
1. successful driving without intereference,
1. collisions due to network interference without TSN features,
1. TSN features enable a successful drive with network interference.

https://user-images.githubusercontent.com/88086083/140656406-81919e7b-8d37-4a7a-a331-be7cd32f6673.mp4

As illustrated below, this demo uses three machines connected to a TSN Ethernet switch, imitating a robot sharing Ethernet links for streams with different criticality levels.
The components in grey are used for performance measurement, which we descrbe in deatail in the measurement section below.
`Machine C` runs the Gazebo simulation. The control of the modeled vehicle runs on an embedded controller `machine A` and publishes the safety-critical topic `/command` based on the data from the `/odometry` topic.
An interference `machine B` floods the egress of `port 3` and interfere with the control traffic in the `/command` topic.
This interference is likely to trigger a collision in the simulation.
Interference may originate from a bug in `machine B`, see the bug icon, or from a network design sharing an Ethernet link between traffic streams with different criticality levels, see the fire icon.
Fortunately, if we link the safety-critical DDS topic `/command` to a TSN stream with a high priority using
[IEEE 802.1Q Priority-Based Scheduling (`PBS`)](https://en.wikipedia.org/wiki/Time-Sensitive_Networking#Scheduling_and_traffic_shaping), then the vehicle completes the moose test successfully.
Furthermore, we can de-burst the interference traffic using the TSN's protocol
[IEEE 802.1Qav Credit-Based Shaper (`CBS`)](https://en.wikipedia.org/wiki/Time-Sensitive_Networking#AVB_credit-based_scheduler) to ensure its egress bandwidth is limited.

<p align="center"><img src="images/dds_tsn_mini_demo.png" alt="simplified demo architecture" width="700" class="center"/></p>


The DDS-TSN mapping demo instructions below leverage the DDS XML profiles for [Connext DDS](dds_tsn_demo/src/dds_tsn_profile_connext.xml) and [Fast DDS](dds_tsn_demo/src/dds_tsn_profile_fastdds.xml). The XML files bind the DDS communication sockets to the VLAN interface, which has a built-in VLAN tagging rule assigning the outgoing traffic a higher priority, as we describe in configuration `Option A`. Another option is to map the DSCP/TOS filed in the IP header to the VLAN PCP value, which we describe in configuration `Option B`.

## Prerequisites
- Three machines with Ubuntu 20.04, machines A and B can be embedded ARM-based systems, machine C will benefit from a discrete GPU.
- A TSN-capable Ethernet switch with PCP and VLAN support included in IEEE 802.1Q-2014 and onwards. For example, the NXP [SJA1110](https://www.nxp.com/products/interfaces/ethernet-/automotive-ethernet-switches/multi-gig-safe-and-secure-tsn-ethernet-switch-with-integrated-100base-t1-phys:SJA1110). In our experiment, we use the SJA1110 switch on the [S32G-VNP-RDB](https://www.nxp.com/design/designs/s32g-reference-design-for-vehicle-network-processing:S32G-VNP-RDB) board, which is the S32G reference design for vehicle network processing.
- ROS2 Foxy base and `iproute2` for the `tc` command on machine A:
    follow the official [ROS2 installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#set-locale) to install ROS2 Foxy *base*.
    Then install other dependencies:
    ```bash
    sudo apt install -y python3-colcon-ros iproute2
    ```
- To use configuration Option B described in the *Configuration* section below, build the following kernel modules for machine A to enable Linux Traffic Control (tc) actions, packet classification, and U32 filter for outgoing packets:
    ```
    CONFIG_NET_CLS_ACT=y
    CONFIG_NET_CLS_FLOW=y
    CONFIG_NET_CLS_FLOWER=y
    CONFIG_NET_CLS_U32=y
    CONFIG_CLS_U32_MARK=y
    ```
  In our experiment, the machine A runs on the i.MX 8M [NavQ](https://nxp.gitbook.io/8mmnavq/) Linux companion computer platform. The NavQ kernel can be configured and built following instructions [here](https://github.com/NXPmicro/meta-nxp-hovergames/tree/demo).
- `iperf3` on machine B:
    ```bash
    sudo apt install -y iperf3
    ```
- ROS2 Foxy and Gazebo on machine C:
    follow the official [ROS2 installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#set-locale) to install ROS2 Foxy *desktop*.
    Then install Gazebo and other dependencies:
    ```bash
    sudo apt install -y python3-colcon-ros ros-foxy-gazebo-ros ros-foxy-gazebo-plugins iperf3 iproute2
    ```

## Installation
1. Our demonstration supports the Fast DDS, which is pre-installed and the default DDS middleware layer in ROS2 Foxy, and the RTI Connext DDS. The RTI Connext DDS can be installed by following the documentation [here](https://github.com/ros2/rmw_connextdds) on machines A and C.
    - For an Intel machine:
       ```bash
       # install free debian packages for Connext DDS on Intel machine
       sudo apt install -y ros-foxy-rmw-connext-cpp
       source /opt/ros/foxy/setup.bash
       git clone https://github.com/rticommunity/rmw_connextdds.git ~/rmw_connextdds
       cd ~/rmw_connextdds
       git checkout foxy
       colcon build
       # run the following commands in each new shell to choose this DDS distribution
       source ~/rmw_connextdds/install/setup.sh
       export RMW_IMPLEMENTATION=rmw_connextdds
       ```
    - For an ARM machine: the free Debian package of Connext DDS is not available for `arm64`, however, you can download Connext DDS Professional from [here](https://www.rti.com/products) and build it on the `arm64` target.
1. Build the code from the repository root directory (with this README) on machines A and C. If you use the Connext DDS, set the environment as described in the previous step.
    ```bash
    git clone https://github.com/NXP/dds-tsn.git
    cd dds-tsn
    source /opt/ros/foxy/setup.bash
    colcon build
    source install/setup.sh
    ```

## Configuration

### Option A: VLAN-to-PCP mapping using egress-qos-map
No specific Linux kernel modules are required for this option.
On machine A, create a VLAN interface with the IP address ending with `.2`:
```bash
./scripts/make_vlan.bash
```
Most likely you'll need to override network variables in the script for your system. For example, to use the physical interface `eth0`, netmask `10.10.*.*`, and a specific egress-qos-map value (other than the default `0:5`):
```bash
PIF=eth0 NETMASK=10.10 EGRESS_QOS_MAP="egress-qos-map 0:4"  ./scripts/make_vlan.bash
```

### Option B: DSCP-to-PCP mapping with traffic control filter
For this option, machine A needs specific kernel configuration, see details in *Prerequisites* section above.
The DDS distribution in use should support *TransportPriority* QoS policy.
At the moment of writing this README, [Fast DDS did not support this feature](https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/core/policy/transportpriorityqospolicy.html) and we used RTI's Connext DDS on `arm64` for the experiment.
Alternatively, one can use machine A with an Intel processor and the free Debian package for [RTI Connext DDS](https://www.rti.com/products).

Configure VLAN interfaces with the IP address ending with `.2`. Assuming your setup uses the physical interface `eth0` and netmask `10.10.*.*`, we setup a VLAN interface that filters on a given TOS value of `0x14`:
```bash
PIF=eth0 NETMASK=10.10 OPTION_B=on ./scripts/make_vlan.bash
# to verify the configuration, send iperf3 streams to machine C and check the packet TOS and PCP value using Wireshark
iperf3 -c MACHINE_C_VLAN_INTERFACE -u -S 0x14 -t20
```

### Common configuration
1. On machine C, create a virtual interface with IP addresss ending with `.3`, for example:
   ```bash
   PIF=eth0 NETMASK=10.10 IP_SUFFIX=3 ./scripts/make_vlan.bash
   ```
1. Make sure you can ping the virtual interfaces on both machine A and C:
   ```bash
   ping -c 3 10.10.30.2 # ping machine A VLAN interface
   ping -c 3 10.10.30.3 # ping machine C VLAN interface
   ```
1. TSN switch configuration

    TSN switches need to be configured to allow traffic from a given VLAN on specific switch ports. For the NXP SJA1110 it can be done in the SDK available for download [here](https://www.nxp.com/products/interfaces/ethernet-/automotive-ethernet-switches/multi-gig-safe-and-secure-tsn-ethernet-switch-with-integrated-100base-t1-phys:SJA1110). VLAN ports membership on off-the-shelf managed switches often can be configured using a web interface.

    To configure VLAN on the NXP SJA1110 switch, add VLAN 30 to the membership fields of all the switch ports. In the SJA1110 SDK GUI open the `Peripheral` configuration, select the switch fabric, then click on `VLAN Lookup Table` dialogue. Then tick all ports in the section `VMEMB_PORT`, all ports in the section `BC_DOMAIN`, all ports in `TAG_PORT` and, finally, set the `VLANID` to 30.

    To make the effect of the DDS-TSN integration easily visible in this demo, configure the switch to limit the link speed of the `vehicle_control command` to `100 Mbps` or lower. Otherwise, the moose test may always succeed even with interference enabled.

### Configuration of the XML files
In short, you may need to edit the IP address or subnet in the XML `allow_interfaces_list` field for DDS Connext and the `interfaceWhiteList` field for Fast DDS to match your local VLAN networking interface.
 
If there are multiple interfaces on a machine, we need a mechanism to steer the DDS traffic to a preferred network interface to ensure the Ethernet packets are tagged with an appropriate VLAN ID and the priority code point (PCP). Then the PCP and VLAN ID will enable TSN features in the networking devices, such as switches, for the packets. The XML profiles can define on which local interface the DDS middleware communicates. To find it out the IP address of the local VLAN interface, you can run `ifconfig` and find the IP address of the created VLAN interface on the local machine. For example, in the demo video at 0m:46s you can see how we added a VLAN interface `enp0s31f6.30` on machine C, which got `192.168.30.1`. On machine A, the VLAN interface is `eth0.30` and interface’s IP is `192.168.30.2`, as can be seen in the console around 01m:06s. So, on machine C we’ll need to set the `interfaceWhiteList` tag to `192.168.30.1`, and on machine A to `192.168.30.2`. For the DDS Connext we need to use a subnet instead of the IP address of the local interface in the `allow_interfaces_list`.

Note that in the demo video the IPs are different than examples of using the `make_vlan.sh` script in the README. 

## Execution
1. Start the `iperf3` server on machine C:
    ```bash
    iperf3 -s > /dev/null 2>&1 &
1. Start Gazebo on machine C, the vehicle will remain still:
    ```bash
    ros2 launch dds_tsn_demo world_launch.py
    ```
1. Start the controller on machine A to drive the vehicle in the simulator using *only the physical interface*. Make sure the VLAN interfaces are down or deleted. Access via `ssh` can be heavily interfered by the interference stream introduced during the test:
    ```bash
    unset NDDS_QOS_PROFILES # disable XML profile for Connext DDS
    unset FASTRTPS_DEFAULT_PROFILES_FILE # disable XML profile for FastDDS
    ros2 launch dds_tsn_demo control_launch.py
    ```
1. When the vehicle starts moving, start interference on machine B towards the physical network interface of machine C:
    ```bash
    iperf3 -c MACHINE_C_IP -u -l53K -b1G -t600 # adapt here the IP address of the physical interface on machine C
    ```
1. The vehicle is likely to crash into the obstacles (the pedestrian and the other vehicle) or drive off the road in Gazebo. Close the Gazebo simulator and kill the vehicle control from step 4.
1. Let's leverage the DDS-TSN integration to mitigate interference by coupling DDS traffic to TSN VLAN tags, which will prioritize the data transmission. Restart the Gazebo by following step 2. Then, on machine A, export the DDS profile XML, bring up the virtual network, and run the control node:
    ```bash
    export NDDS_QOS_PROFILES=$(pwd)/dds_tsn_demo/src/dds_tsn_profile_connext.xml # use Connext DDS to map a DDS topic to a specific TOS/DSCP
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/dds_tsn_demo/src/dds_tsn_profile_fastdds.xml # for Fast DDS
    ros2 launch dds_tsn_demo control_launch.py
    ```
1. Now start the interference as described in step 4.
1. The vehicle should be able to successfully finish the moose test in the Gazebo simulation thanks to prioritized vehicle control traffic.

## How to measure TSN performance

To accurately measure the TSN performance of the network, consider [installing gPTP time synchronization on machines A and C](https://tsn.readthedocs.io/timesync.html). Furthermore, check if your network interfaces perform hardware time stamping with `sudo ethtool -T <network_interface>`.
The measurement setup is shown in the block diagram above in grey, where *HW TS* stands for hardware timestamping.

Commands to be run on each machine is introduced below; for more information on `step 1` and `step 2`, see [traffic_analysis README](tools/traffic_analysis/README.md):

1. Run `tshark` with timestaping on machine A and C during the dds-tsn demo. After the demo is over move the `machine_a.pcapng` file to machine C.
   ```bash
   # on machine A
   tshark -i <interface> --time-stamp-type adapter_unsynced -w machine_a.pcapng
   # on machine C
   tshark -i <interface> --time-stamp-type adapter_unsynced -w machine_c.pcapng
   ```
1. On machine C, run `traffic_analysis.py` script on both `.pcapng` files, save the results to a `.csv` file. Use the UDP source port (here 46278) to filter for UDP datagram only related to the `/cmd_vel` topic of Gazebo:
   ```bash
   $ python3 tools/traffic_analysis/traffic_analysis.py -p machine_a.pcapng -c rtps.data machine_a.csv -f 'udp.srcport == 46278'
   $ python3 tools/traffic_analysis/traffic_analysis.py -p machine_c.pcapng -c rtps.data machine_c.csv -f 'udp.srcport == 46278'
   ```
1. Merge both .csv files with [rtps_csv_merge.py](tools/rtps_csv_merge/rtps_csv_merge.py):
   ```bash
   $ python3 tools/rtps_csv_merge/rtps_csv_merge.py machine_a.csv machine_c.csv merged.csv
   ```
   The generated `merged.csv` will then contain the HW timestamp from both the sending and the receiving side for a specific RTPS sequence number. This can be used for further processing.

### Calibration to reproduce the vehicle crash on various platforms 
If the moose test passes with interference, you may need to tune the demo to your platform. With the help of various users of our repository, we derived the following recommendations to reproduce the crash due to interference:
1. Double-check you followed the instructions, including the reduction of the Ethernet link speed down to 100Mbps or lower.
1. Increase the severity of delaying/dropping a control packet due to interference by increasing the time period between control messages. In [our code](dds_tsn_demo/src/vehicle_control.cpp#L45), the period is 10ms now. By increasing it gradually on machine A, one can get to the point when the moose test starts to fail. Once this threshold is found, the sensitivity of the interference should become higher.
1. Tweak the vehicle steering control in the [steeringCommands](dds_tsn_demo/src/vehicle_control.cpp#L68) table. The table includes a list of ego vehicle’s longitudinal coordinates and associated linear and angular components of the velocity applied at this coordinate to the ego vehicle by the vehicle controller. By varying the coordinates, we can modify where in the trajectory the velocity of the ego vehicle changes. Changing the velocity components themselves adjusts the motion changes at a given location.
1. Modify the simulated scene in the [modeled world](dds_tsn_demo/world/gazebo_diff_drive_moose_test.world) in terms of object position, size and mass to make the timing of control packets even more critical. Modification of the world can be easily done in the [Gazebo simulator GUI](https://classic.gazebosim.org/tutorials?tut=build_world&ver=1.9).
1. Lower the CPU clock speed, especially, if you use desktop- or server-grade processors instead of embedded SoCs, see email Thursday, March 23, 2023 8:45 AM

## Troubleshooting
1. If you get an error `colcon: command not found`, run `sudo apt install python3-colcon-common-extensions`.
1. If you get an error while starting Gazebo `X Error of failed request: BadValue (integer parameter out of range for operation)` try rebooting your machine. 
1. If you can't start Gazebo due to an error `[gazebo-1] [Err] [Master.cc:95] EXCEPTION: Unable to start server[bind: Address already in use]. There is probably another Gazebo process running.`, run `killall gzserver gzclient`.
1. During ROS installation, apt update fails due to ROS repository public key issues. To resolve it, run the commands below:
    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    ```

## Useful links
1. [Free S32G webinar on DDS-TSN integration in the Autoware.auto Autonomous Valet Parking application](https://www.nxp.com/design/training/dds-and-tsn-where-software-and-hardware-meet-for-dependable-communication-using-rti-connext-drive-and-nxp-s32g-processor:TIP-DDS-AND-TSN-WHERE-SOFTWARE-AND-HARDWARE-MEET)
1. [Driving Interoperability and Performance in Automotive Systems with DDS and TSN](https://www.nxp.com/webapp/Download?colCode=DDSTSNWP) - DDS-TSN white paper co-authored by NXP and RTI
1. [ROS2 presentation about DDS and TSN](https://drive.google.com/file/d/1_PvBvFd4--kLCUn3R037U-31I2E0BujN/view?usp=sharing). Here is our [presentation recording](https://drive.google.com/file/d/15jU5VVkBa7ULnnnXSDOvKT6qn516CCAR/view?usp=sharing). More about the topic, please check [the discussion thread on ROS2 Discourse](https://discourse.ros.org/t/ros2-tsn-talk-during-ros2-real-time-wg-regular-call-on-tue-1st-of-feb-16-00-cet/24099/4).
1. [System Archiecture Study Group (SASG) presentation on middleware](https://www.sasg.nl/SASG_72_Roly-poly_on_middleware.pdf)
1. https://tsn.readthedocs.io/index.html - hands-on tutorial on TSN and VLAN support in GNU/Linux
1. https://arxiv.org/pdf/1808.10821.pdf - excellent description of the GNU/Linux traffic control and its application in robotics
1. https://wiki.archlinux.org/title/VLAN - VLAN support in GNU/Linux
1. https://tldp.org/HOWTO/Adv-Routing-HOWTO/index.html - Linux Advanced Routing & Traffic Control HOWTO
1. https://en.wikipedia.org/wiki/Type_of_service - Type of Service field in the IP header

## TODO:
1. Change the name of the topics in the C++ and .world to match the illustration in README
1. Describe the CBS configuration of the TSN switch

## License
This software is distributed under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0).
