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

# 20秒待つ
(sleep 20; ros2 topic pub -1 /medicine_response system_msgs/msg/Response "{response: 1}") &

timeout 30 ros2 launch mypkg medicine_rm.launch.py > /tmp/medicine_reminder.log || [ $? -eq 124 ]

#判定
echo "--- Test Results Analysis ---"

#通信が成立するか
grep "status:" /tmp/medicine_reminder.log

#状態遷移の全パターン確認
grep "status: 1" /tmp/medicine_reminder.log  # NORMALになったか
grep "status: 2" /tmp/medicine_reminder.log  # URGENTになったか
tail -n 20 /tmp/medicine_reminder.log | grep "status: 0"  # 最後にOFFに戻ったか

#ステータス重複を排除できたか
CHANGE_COUNT=$(grep -o "status: [0-2]" /tmp/medicine_reminder.log | uniq | wc -l)
echo "Status change count: $CHANGE_COUNT"
[ "$CHANGE_COUNT" -ge 4 ]

#目視確認用のログも出力
grep "status:" /tmp/medicine_reminder.log | uniq -c

echo "--- All Tests Passed! ---"
