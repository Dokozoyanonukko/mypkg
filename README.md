# mypkg : 服薬支援デモシステム

服用イベントの発生から完了までを監視し、ユーザ応答と経過時間に応じて3段階の通知ステータス（待機=OFF・通常=NOMAL・緊急=URGENT）をパブリッシュする服薬支援デモシステム

![test](https://github.com/Dokozoyanonukko/mypkg/actions/workflows/test.yml/badge.svg)

## パッケージの目的と責務
本パッケージは服薬状態を判定するロジックを提供し、イベント発生とユーザ応答はデモ用ノードで模擬しています。
判定結果はトピックとして公開され、表示・通知・記録などの具体的な処理は外部ノードとの連携を想定しています。

## ノード構成
| ノード名 | 役割 |
|:---:|:---:|
| event | 服薬イベントの発生（デモ用） |
| response | ユーザ応答（無反応・服薬完了）の模擬（デモ用） |
| judge | イベントと応答からの状態判定（本パッケージの中核ノード） |
| status | 判定結果の受取、状態の出力 |

> [!NOTE]
> eventとresponseはデモ用実装であり、実運用では外部ノードによる置換を想定しています。

## トピック一覧
| ノード名 | Publish/Subscribe | トピック名 | 型 | 内容 | 
|:---:|:---:|:---:|:---:|:---:|
| event | Publish | **/reminder_event** | **system_msgs/msg/RmEvent** |イベント発行|
| response | Publish | **/response** | **system_msgs/msg/Response** |ユーザ応答|
| judge | Subscribe<br/>Subscribe<br/>Publish | /reminder_event<br/>/response<br/>**/reminder_command** | system_msgs/msg/RmEvent<br/>system_msgs/msg/Response<br/>**system_msgs/msg/RmNotioncmd** | <br/><br/>判定結果|
| status | Subscribe<br/>Publish | /reminder_command<br/>**/reminder_status** | system_msgs/msg/RmNotioncmd<br/>**system_msgs/msg/RmStatus** | <br/>ステータス|

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

#以下出力例（一部抜粋）
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

## デモ内容説明
1.  event.pyが5 秒ごとに服薬イベント（服薬時間アナウンス）をパブリッシュ
2.  response.pyがユーザ応答を模擬し、5 秒ごとにNO_RESPONSEまたはTAKEN信号をパブリッシュ（TAKENは20秒に1回）
3.  judge.pyがこれら2つのメッセージをサブスクライブし、服薬イベント発生から10秒以上たってもTAKENを受信できなかった場合、「服薬忘れ」とみなしてステータスを緊急状態へと引き上げる
4.  status.pyがステータス情報を受け取り、現在のステータスを出力する
5.  以上の流れを繰り返す

## 外部連携時の最小構成例
外部ノードは、以下の2点を実装することで本パッケージの中核となる判定ロジックを利用できます。

* /reminder_eventに服薬イベントをPublish
* /responseに服薬完了（TAKEN）をPublish

判定結果は/reminder_statusから取得できます。

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
