#!/bin/bash
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

set -e

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc

#全ノードを実行して結果を保存
timeout 15 ros2 launch medicine_reminder reminder.launch.py > /tmp/medicine_reminder.log

#reminder_statusが流れている(ノード通信が成立している)かのテスト
cat /tmp/medicine_reminder.log | grep "status:"

#初期状態が存在するか
cat /tmp/medicine_reminder.log | grep "status: 0"

#URGENTが出る （judgenodeが機能している）か
cat /tmp/medicine_reminder.log | grep "status: 2"

#URGENTが最低1回以上起きるか
[ $(grep "status: 2" /tmp/medicine_reminder.log | wc -l) -ge 1 ]

#OFFに戻る（ユーザ応答が反映される）か
cat /tmp/medicine_reminder.log | grep "status: 0"

#同じ命令が連続しない設計が機能しているか
grep "status:" /tmp/medicine_reminder.log | uniq -c


