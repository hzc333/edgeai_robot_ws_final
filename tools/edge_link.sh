#!/usr/bin/env bash
# Copyright 2026 EdgeAI Robotics Challenge Team
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
#
# Run a command as the edge node, with the robots on the other end of an emulated network.
#
# Two private network namespaces (no root needed: a user namespace) joined by a veth pair:
# the edge node at 10.77.0.1 runs COMMAND, the robots at 10.77.0.2 run whatever COMMAND
# starts through $EDGEAI_ROBOT_EXEC (tools/midterm_rehearsal.py --split does). netem
# shapes both directions; multicast works on the link unless the middleware turns it off.
#
#   pixi run bash -c 'source install/setup.bash && tools/edge_link.sh --rmw zenoh \
#       --delay-ms 15 --jitter-ms 5 -- python tools/midterm_rehearsal.py --split'
#
# --rmw cyclone          CycloneDDS as configured by default: multicast discovery, and
#                        multicast data for every topic with several remote readers, even
#                        when all of them are on the robot (a camera stream floods the link)
#       cyclone-spdp     CycloneDDS with multicast only for discovery (AllowMulticast spdp)
#       cyclone-unicast  CycloneDDS without multicast; discovery through the two addresses
#       zenoh            rmw_zenoh: one router per side, the robots' router connects to the
#                        edge router (one TCP connection, port 7447)
# --delay-ms/--jitter-ms  one-way delay and jitter per direction (default 0)
# --loss-pct              packet loss per direction (default 0)
# --rate-mbit             bandwidth per direction (default unlimited)
# --outage START:SECONDS  drop every packet for SECONDS, START seconds after COMMAND
#                         started (repeatable); prints when the link goes down and up
#
# At the end it prints the bytes and packets that crossed the link in each direction.

set -euo pipefail

EDGE_IP=10.77.0.1
ROBOT_IP=10.77.0.2
rmw=cyclone delay=0 jitter=0 loss=0 rate=
outages=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --outage) [[ $2 =~ ^[0-9]+:[0-9]+$ ]] || { echo "--outage needs START:SECONDS" >&2; exit 2; }
              outages+=("$2"); shift 2 ;;
    --rmw) rmw=$2; shift 2 ;;
    --delay-ms) delay=$2; shift 2 ;;
    --jitter-ms) jitter=$2; shift 2 ;;
    --loss-pct) loss=$2; shift 2 ;;
    --rate-mbit) rate=$2; shift 2 ;;
    --) shift; break ;;
    *) echo "unknown option $1 (see the header of $0)" >&2; exit 2 ;;
  esac
done
[[ $# -gt 0 ]] || { echo "usage: $0 [options] -- COMMAND..." >&2; exit 2; }
case $rmw in cyclone|cyclone-spdp|cyclone-unicast|zenoh) ;; *) echo "unknown --rmw $rmw" >&2; exit 2 ;; esac

if [[ -z ${EDGEAI_LINK_INSIDE:-} ]]; then
  exec unshare --user --map-root-user --net --kill-child \
    env EDGEAI_LINK_INSIDE=1 "$0" --rmw "$rmw" --delay-ms "$delay" --jitter-ms "$jitter" \
    --loss-pct "$loss" ${rate:+--rate-mbit "$rate"} \
    $(for o in "${outages[@]}"; do printf -- '--outage %s ' "$o"; done) -- "$@"
fi

# Inside: this namespace is the edge node; a second one holds the robots.
ip link set lo up
unshare --net sleep infinity &
robot_ns=$!
pids=("$robot_ns")
cleanup() { kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT
sleep 0.2
robot() { nsenter -t "$robot_ns" -n -- "$@"; }
ip link add edge0 type veth peer name robot0
ip link set robot0 netns "$robot_ns"
ip addr add "$EDGE_IP/24" dev edge0
ip link set edge0 up
robot ip link set lo up
robot ip addr add "$ROBOT_IP/24" dev robot0
robot ip link set robot0 up
# Multicast (DDS and Gazebo discovery) needs a route.
ip route add 224.0.0.0/4 dev edge0
robot ip route add 224.0.0.0/4 dev robot0
shape=(netem delay "${delay}ms" "${jitter}ms" loss "${loss}%" ${rate:+rate "${rate}mbit"})
tc qdisc add dev edge0 root "${shape[@]}"
robot tc qdisc add dev robot0 root "${shape[@]}"
ping -c 3 -q "$ROBOT_IP" | tail -1 | sed 's/^/link /'

case $rmw in
  cyclone)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ;;
  cyclone-spdp)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast>\
</General><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery>\
</Domain></CycloneDDS>" ;;
  cyclone-unicast)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>\
</General><Discovery><ParticipantIndex>auto</ParticipantIndex>\
<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address=\"$EDGE_IP\"/>\
<Peer address=\"$ROBOT_IP\"/></Peers></Discovery></Domain></CycloneDDS>" ;;
  zenoh)
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    zenohd=$(ros2 pkg prefix rmw_zenoh_cpp)/lib/rmw_zenoh_cpp/rmw_zenohd
    "$zenohd" >/dev/null 2>&1 &
    pids+=($!)
    ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/$EDGE_IP:7447\"]" \
      robot "$zenohd" >/dev/null 2>&1 &
    pids+=($!)
    sleep 2 ;;
esac
export EDGEAI_ROBOT_EXEC="nsenter -t $robot_ns -n --"

counters() {   # tx bytes, rx bytes, tx packets, rx packets of edge0 (/sys shows the host)
  awk '$1 == "edge0:" { print $10, $2, $11, $3 }' /proc/net/dev
}
read -r tx0 rx0 txp0 rxp0 < <(counters)
start=$SECONDS
status=0
"$@" &
command_pid=$!
if [[ ${#outages[@]} -gt 0 ]]; then
  (
    for outage in "${outages[@]}"; do
      at=${outage%%:*} length=${outage##*:}
      while (( SECONDS - start < at )); do sleep 0.5; done
      tc qdisc change dev edge0 root netem loss 100%
      robot tc qdisc change dev robot0 root netem loss 100%
      echo "link down at $((SECONDS - start)) s for $length s ($(date -u +%H:%M:%S) UTC)"
      sleep "$length"
      tc qdisc change dev edge0 root "${shape[@]}"
      robot tc qdisc change dev robot0 root "${shape[@]}"
      echo "link up at $((SECONDS - start)) s ($(date -u +%H:%M:%S) UTC)"
    done
  ) &
  pids+=($!)
fi
wait "$command_pid" || status=$?
read -r tx1 rx1 txp1 rxp1 < <(counters)
seconds=$((SECONDS - start))
awk -v rmw="$rmw" -v s="$seconds" -v er=$((tx1 - tx0)) -v re=$((rx1 - rx0)) \
    -v erp=$((txp1 - txp0)) -v rep=$((rxp1 - rxp0)) 'BEGIN {
  printf "link %s over %d s: edge->robot %.1f MiB (%.1f KiB/s, %d packets), " \
         "robot->edge %.1f MiB (%.1f KiB/s, %d packets)\n", rmw, s, er / 1048576,
         er / 1024 / (s > 0 ? s : 1), erp, re / 1048576, re / 1024 / (s > 0 ? s : 1), rep }'
exit $status
