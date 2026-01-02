#!/bin/bash
# SPDX-FileCopyrightText: 2025 Shoma Takatori
# SPDX-License-Identifier: BSD-3-Clause

ng () {
    echo "${1}行目が違うよ"
    res=1
}

res=0

source /opt/ros/humble/setup.bash

[ -d build ] || colcon build
source install/setup.bash

### NORMAL INPUT ###
out=$(timeout 5s ros2 run mypkg timer 2>&1 | grep "Pomodoro" || true)

if [ "${out}" = "" ]; then
    ng "$LINENO"
fi

### NODE LIST CHECK ###
ros2 run mypkg timer > /dev/null 2>&1 &
node_pid=$!

sleep 5

ros2 node list | grep -q "/pomodoro_timer" || ng "$LINENO"

kill $node_pid

[ "${res}" = 0 ] && echo OK
exit $res
