# mypkg-服薬支援デモシステム

服用イベントの発生から完了までを監視し、ユーザ応答と経過時間に応じて3段階の通知ステータス（待機・通常・緊急）をパブリッシュする服薬支援デモシステム

![test](https://github.com/Dokozoyanonukko/mypkg/actions/workflows/test.yml/badge.svg)

# 使い方
リポジトリをコピーしたいディレクトリにて以下の操作を行ってください。

```bash
$ git clone git@github.com:Dokozoyanonukko/mypkg.git
$ cd ~/mypkg
$ rm -rf build install log
$ colcon build --packages-select system_msgs
$ colcon build --packages-select mypkg
$ source install/setup.bash
$ ros2 launch mypkg medicine_rm.launch.py

#以下出力例
[INFO] [launch]: All log files can be found below /home/nagi/.ros/log/2026-01-02-03-15-08-275050-MSI-431824
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [event-1]: process started with pid [431838]
[INFO] [judge-2]: process started with pid [431839]
[INFO] [status-3]: process started with pid [431840]
[INFO] [response-4]: process started with pid [431841]
[judge-2] [INFO] [1767291309.094442677] [judge]: Judge Node started. Watching for medication events...
[judge-2] [INFO] [1767291314.082885009] [judge]: Status: NORMAL (Waiting for TAKEN)
[event-1] [INFO] [1767291314.098865717] [event]: Medication Event Sent! (event=1)
[event-1] [INFO] [1767291319.082469814] [event]: Medication Event Sent! (event=1)
[event-1] [INFO] [1767291324.085631331] [event]: Medication Event Sent! (event=1)
[judge-2] [WARN] [1767291324.089481911] [judge]: Status: URGENT (10.0s elapsed!)
[event-1] [INFO] [1767291329.084753546] [event]: Medication Event Sent! (event=1)
[judge-2] [INFO] [1767291330.095119403] [judge]: Status: OFF (Medication Taken)
```

# デモ内容説明
1. event.pyが5 秒ごとに服薬イベント（服薬時間アナウンス）をパブリッシュ
2. response.pyがユーザ応答を模擬し、5 秒ごとにNO_RESPONSEまたはTAKEN信号をパブリッシュ
3. judge.pyがこれら2つのメッセージをサブスクライブし、服薬イベント発生から10秒以上たってもTAKENを受信できなかった場合、「服薬忘れ」とみなしてステータスを緊急状態へと引き上げる
4. status.pyがステータス情報を受け取り、現在のステータスを出力する
5. 以上の流れを繰り返す

> [!NOTE]
> 本パッケージはデモシステムのため、服薬間隔が5秒、ユーザの服薬を示す 応答(TAKEN)は20秒に1回に設定しています。

# 必要なソフトウェア
* Python
    * テスト済みバージョン：3.10 ~ 3.12
* ROS2
    * 動作確認済みバージョン：Jazzy
* CMake
    * テスト済みバージョン：3.26 ~ 3.28

# テスト環境
* Ubuntu 24.04 LTS

# ライセンス
* このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます．
* © 2025 Dokozoyanonukko
