#!/bin/bash
# MM3 test (d): the twin's ROS topics keep publishing while G1_CONTROL=lowcmd.
#
# The point of the test is that taking the articulation away from the locomotion
# policy must not silence the sensor half of the twin -- lowcmd changes who drives
# the joints, not whether the robot is a robot. Rates are checked against the
# contract, not against round numbers.
#
# TWIN_CAMERA=0 on this box (C-23), so the camera topics are expected absent and are
# reported as such rather than counted as failures.
set -u
NS="${NS:-/ferox/g1_01}"
DOMAIN="${ROS_DOMAIN_ID:-42}"
DUR="${DUR:-8}"
TOPICS=("$NS/odom:51.4" "$NS/imu/data:93.7" "$NS/scan:10.0" "/livox/lidar:10.0" "/livox/imu:200.0" "/tf:100.0")

echo "== MM3 (d) ROS topics under G1_CONTROL=lowcmd (domain $DOMAIN, ${DUR}s each) =="
docker run --rm --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///tmp/ferox_cyclone.xml \
  -v /tmp/ferox-isaac-demo/cyclonedds.xml:/tmp/ferox_cyclone.xml:ro \
  ferox/nav:humble bash -lc '
    source /opt/ros/humble/setup.bash
    echo "--- topic list ---"
    timeout 15 ros2 topic list 2>/dev/null | sort
    for spec in '"${TOPICS[*]}"'; do
      t="${spec%%:*}"; want="${spec##*:}"
      got=$(timeout '"$((DUR+6))"' ros2 topic hz "$t" --window 50 2>/dev/null \
            | grep -m1 "average rate" | awk "{print \$3}")
      printf "%-28s want %8s Hz   got %10s Hz\n" "$t" "$want" "${got:-NONE}"
    done
  '
