#!/bin/bash
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

set -e
source /opt/ros/jazzy/setup.bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source install/setup.bash

#サブスクライバが現れるまで最大10秒待機
(sleep 22; ros2 topic pub --wait 10 -1 /medicine_response system_msgs/msg/Response "{response: 1}") > /dev/null &

#実行
timeout 35 ros2 launch mypkg medicine_rm.launch.py > /tmp/medicine_reminder.log || [ $? -eq 124 ]

#判定
echo "--- Starting Formal Verification ---"

#なんらかのステータスがパブリッシュされるか
echo -n "Test 1: Node Communication... "
grep -q "status:" /tmp/medicine_reminder.log && echo "PASSED"

#初期状態が０か
echo -n "Test 2: Initial State (OFF)... "
grep -q "status: 0" /tmp/medicine_reminder.log && echo "PASSED"

#EVENTからの信号でステータスが1になるか
echo -n "Test 3: Event Detection (NORMAL)... "
grep -q "status: 1" /tmp/medicine_reminder.log && echo "PASSED"

#URGENTが正しく発動するか
echo -n "Test 4: Alert Logic (URGENT)... "
grep -q "status: 2" /tmp/medicine_reminder.log && echo "PASSED"

#TAKEDを受けてステータスが０に戻るか
echo -n "Test 5: Recovery Logic (BACK TO OFF)... "
tail -n 20 /tmp/medicine_reminder.log | grep -q "status: 0" && echo "PASSED"

#ステータスの重複がないか
echo -n "Test 6: Message Deduplication (uniq)... "
CHANGE_COUNT=$(grep -o "status: [0-2]" /tmp/medicine_reminder.log | uniq | wc -l)
if [ "$CHANGE_COUNT" -ge 3 ]; then
    echo "PASSED (Changes: $CHANGE_COUNT)"
else
    echo "FAILED"
    exit 1
fi

echo "--- All Tests Completed Successfully ---"
