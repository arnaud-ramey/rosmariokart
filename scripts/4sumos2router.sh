#!/bin/sh

WLAN_INTERFACE="wlp2s0"
WLAN_ESSID="TPSUMO"
WLAN_CON_NAME="TPSUMO"
WLAN_ROUTER_IP="192.168.1.254"
COMMONARGS="$WLAN_INTERFACE  $WLAN_ESSID  $WLAN_ROUTER_IP  $WLAN_CON_NAME"

rosrun rossumo sumo2router_client.bash $COMMONARGS  "JumpingSumo-b017386"  "192.168.1.15" # Arnaud khaki
rosrun rossumo sumo2router_client.bash $COMMONARGS  "JumpingSumo-b173959"  "192.168.1.16" # Arnaud noir
