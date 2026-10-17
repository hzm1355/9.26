#!/bin/bash
HOSTIP=$(ip route show default | awk '{print $3}')
[ -z "$HOSTIP" ] && HOSTIP=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
echo "host ip: $HOSTIP"
for d in ":0" ":1" "$HOSTIP:0" "$HOSTIP:1"; do
    echo -n "DISPLAY=$d -> "
    timeout 5 env DISPLAY="$d" xdpyinfo >/dev/null 2>&1 && echo OK || echo FAIL
done
