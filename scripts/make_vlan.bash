#!/bin/bash

# input parameters
PIF=${PIF:-eno1} # physical interface name
VLAN=${VLAN:-30}
EGRESS_QOS_MAP=${EGRESS_QOS_MAP:-egress-qos-map 0:5}
NETMASK=${NETMASK:-192.168}
IP_SUFFIX=${IP_SUFFIX:-2}

IP=$NETMASK.$VLAN.$IP_SUFFIX
VLAN_IF=$PIF.$VLAN

OPTION_B=${OPTION_B:-"off"}
TOS=${TOS:-0x14} # full 8 bits for DSCP and ECN fields
PCP=${PCP:-5}

if [ "$OPTION_B" = "on" ]; then
    unset EGRESS_QOS_MAP
fi

echo "Creating virtual network interface $VLAN_IF with $IP $EGRESS_QOS_MAP"
sudo ip link delete $VLAN_IF || echo $VLAN_IF was not present, continuing...
sudo ip link add link $PIF name $VLAN_IF type vlan id $VLAN $EGRESS_QOS_MAP
sudo ip addr add $IP/24 brd $NETMASK.$VLAN.255 dev $VLAN_IF
sudo ip link set dev $VLAN_IF up

if [ "$OPTION_B" = "on" ]
  then
    echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "Option B: DSCP-to-PCP mapping with traffic control filter"
    sudo tc qdisc delete dev $PIF clsact >/dev/null 2>&1
    sudo tc qdisc add dev $PIF clsact
    sudo tc filter add dev $PIF egress protocol 802.1Q u32 match ip tos $TOS 0xff action vlan modify id $VLAN priority $PCP
   else
    echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "Option A: VLAN-to-PCP mapping using egress-qos-map"
    sudo tc filter delete dev $PIF egress >/dev/null 2>&1
fi

echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "Applied egress filters:"
tc -s -d filter show dev $PIF egress
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "New interfaces:"
ip address
