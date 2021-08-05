#!/bin/bash

# input parameters
PIF=${PIF:-eno1} # physical interface name
VLAN=${VLAN:-30}
EGRESS_QOS_MAP=${EGRESS_QOS_MAP:-egress-qos-map 0:5}
NETMASK=${NETMASK:-192.168}
IP_SUFFIX=${IP_SUFFIX:-2}

IP=$NETMASK.$VLAN.$IP_SUFFIX
VLAN_IF=$PIF.$VLAN

echo Creating virtual network interface $VLAN_IF with $IP and $EGRESS_QOS_MAP
sudo ip link delete $VLAN_IF || echo $VLAN_IF was not present, continuing...
sudo ip link add link $PIF name $VLAN_IF type vlan id $VLAN $EGRESS_QOS_MAP
sudo ip addr add $IP/24 brd $NETMASK.$VLAN.255 dev $VLAN_IF
sudo ip link set dev $VLAN_IF up

echo New interfaces
ip address