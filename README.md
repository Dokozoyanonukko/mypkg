# mypkg : 服薬支援デモシステム

服用イベントの発生から完了までを監視し、ユーザ応答と経過時間に応じて3段階の通知ステータス（待機=OFF・通常=NORMAL・緊急=URGENT）をパブリッシュする服薬支援デモシステム

![test](https://github.com/Dokozoyanonukko/mypkg/actions/workflows/test.yml/badge.svg)

## パッケージの機能と責務  
### 機能
本パッケージは、服薬のリマインドとそれに対するユーザ応答を入力として「服薬したかどうか」を判定し、結果をROS2トピックとして出力する機能を提供します。  

### 責務  
本パッケージの責務は、入力内容から服薬したかを判断してトピックを出力することまでです。服薬イベントの発行や判定結果の画面表示のような処理は利用者が外部ノードを用いて自由に組み合わせることを想定しています。  
なお本パッケージには、動作確認用として服薬イベントおよびユーザ応答を模擬するデモ用ノードを含んでいます。

## ノード構成
| ノード名 | 役割 |
|:---:|:---:|
| `event.py` | 服薬イベントの発行（デモ用） |
| `response.py` | ユーザ応答（無反応・服薬完了）の模擬（デモ用） |
| `judge.py` | イベントと応答からの状態判定（本パッケージの中核ノード） |
| `status.py` | 判定結果の受取、状態の出力 |

> [!NOTE]
> `event.py`と`response.py`はデモ用の実装です。実運用では外部ノードによる置換を想定しています。

## トピック一覧
| ノード名 | Publish/Subscribe | トピック名 | 型 | 内容 | 
|:---:|:---:|:---:|:---:|:---:|
| `event.py` | Publish | **`/reminder_event`** | **`system_msgs/msg/RmEvent`** |イベント発行|
| `response.py` | Publish | **`/response`** | **`system_msgs/msg/Response`** |ユーザ応答|
| `judge.py` | Subscribe<br/>Subscribe<br/>Publish | `/reminder_event`<br/>`/response`<br/>**`/reminder_command`** | `system_msgs/msg/RmEvent`<br/>`system_msgs/msg/Response`<br/>**`system_msgs/msg/RmNotioncmd`** | <br/><br/>判定結果|
| `status.py` | Subscribe<br/>Publish | `/reminder_command`<br/>**`/reminder_status`** | `system_msgs/msg/RmNotioncmd`<br/>**`system_msgs/msg/RmStatus`** | <br/>ステータス|

# 使い方
本パッケージをクローンしたいディレクトリにて以下の操作を行ってください。
また、本パッケージを利用するためには別パッケージの**system_msgs**をクローンする必要があります。本パッケージで使用するメッセージ型が定義されています。

* system_msgsについてはこちらをご覧ください。  
https://github.com/Dokozoyanonukko/system_msgs

```bash
$ git clone git@github.com:Dokozoyanonukko/mypkg.git
$ git clone git@github.com:Dokozoyanonukko/system_msgs.git #外部リポジトリの依存パッケージをクローン
$ colcon build
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
[event-1] [INFO] [1767291314.098865717] [event]: Medication Event Sent! (event=1) #服薬イベント（服薬時間アナウンス）のパブリッシュ
[event-1] [INFO] [1767291319.082469814] [event]: Medication Event Sent! (event=1) #イベントは5秒おきに発行されている（デモ用）
[event-1] [INFO] [1767291324.085631331] [event]: Medication Event Sent! (event=1)
[judge-2] [WARN] [1767291324.089481911] [judge]: Status: URGENT (10.0s elapsed!) #10秒経ってもTAKENが受信できなかった。ステータスをURGENT（緊急）に
[event-1] [INFO] [1767291329.084753546] [event]: Medication Event Sent! (event=1)
[judge-2] [INFO] [1767291330.095119403] [judge]: Status: OFF (Medication Taken) #TAKENを受信。ステータスをOFF（待機）に
```

## デモ内容説明
1.  `event.py`が5 秒ごとに服薬イベント（服薬時間アナウンス）をパブリッシュ
2.  `response.py`がユーザ応答を模擬し、5 秒ごとにNO_RESPONSEまたはTAKEN信号をパブリッシュ（TAKENは20秒に1回）
3.  `judge.py`がこれら2つのメッセージをサブスクライブし、服薬イベント発生から10秒以上たってもTAKENを受信できなかった場合、「服薬忘れ」とみなしてステータスをURGENT（緊急状態）へと引き上げる
4.  `status.py`がステータス情報を受け取り、現在のステータスを出力する
5.  以上の流れを繰り返す

## 外部連携時の最小構成例
外部ノードは、以下の2点を実装することで、本パッケージの中核となる服薬状態を判定するロジックを利用できます。

* `/reminder_event`に服薬イベントをPublish
* `/response`に服薬完了（TAKEN）をPublish

判定結果は`/reminder_status`から取得できます。

# 必要なソフトウェア
* Python
    * 動作確認済：3.12.3
* ROS2
    * 動作確認済：Jazzy
* CMake
    * 動作確認済：3.28.3

# テスト環境
* Ubuntu 24.04 LTS

# ライセンス
* このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます．
* © 2025 Dokozoyanonukko
