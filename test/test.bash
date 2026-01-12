#!/bin/bash
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

set -e

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws/src
if [ ! -d system_msgs ]; then
  git clone https://github.com/Dokozoyanonukko/system_msgs.git
fi

cd $dir/ros2_ws
colcon build
source install/setup.bash

#実行
echo "=== Step 2: Running System & Simulating User Response ==="
(sleep 22; ros2 topic pub --wait 10 -1 /medicine_response system_msgs/msg/Response "{response: 1}") > /dev/null &

#タイムアウト
export PYTHONUNBUFFERED=1
timeout 35 ros2 launch mypkg medicine_rm.launch.py > /tmp/medicine_reminder.log 2>&1 || [ $? -eq 124 ]

echo "--- Captured Log Preview ---"
head -n 20 /tmp/medicine_reminder.log

# ログの書き込み完了を少し待つ
sleep 2

#判定
LOG=/tmp/medicine_reminder.log

#通信成立するか
if grep -qi "status:" $LOG; then echo "Test 1: Node Communication... PASSED"; else echo "Test 1: FAILED"; exit 1; fi

#初期状態がOFFか
if grep -qi "status: OFF" $LOG; then echo "Test 2: Initial State (OFF)... PASSED"; else echo "Test 2: FAILED"; exit 1; fi

#イベント発生でNOMALになるか
if grep -qi "status: NORMAL" $LOG; then echo "Test 3: Event Detection (NORMAL)... PASSED"; else echo "Test 3: FAILED"; exit 1; fi

#URGENTが発動するか
if grep -qi "status: URGENT" $LOG; then echo "Test 4: Alert Logic (URGENT)... PASSED"; else echo "Test 4: FAILED"; exit 1; fi

#TAKENでOFFに復帰するか
if tail -n 20 $LOG | grep -qi "status: OFF"; then echo "Test 5: Recovery Logic (BACK TO OFF)... PASSED"; else echo "Test 5: FAILED"; exit 1; fi

#重複ステータスが排除されるか
CHANGE_COUNT=$(grep -oEi "status: (OFF|NORMAL|URGENT)" $LOG | uniq | wc -l)
if [ "$CHANGE_COUNT" -ge 3 ]; then
    echo "Test 6: Message Deduplication (uniq)... PASSED (Changes: $CHANGE_COUNT)"
else
    echo "Test 6: FAILED (Changes: $CHANGE_COUNT)"
    echo "--- LOG CONTENT ---"
    cat $LOG
    exit 1
fi
echo "--- All Tests Completed Successfully ---"
