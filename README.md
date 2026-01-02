# ポモドーロタイマー 
![test](https://github.com/Shouuuuuu/mypkg/actions/workflows/test.yml/badge.svg)

# Explanation
このパッケージは,ROS 2 Humble環境で動作するポモドーロタイマーを提供する

# Install method
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/Shouuuuuu/mypkg.git
$ cd ~/ros2_ws
$ colcon build
$ source install/setup.bash
```

# Usage
二つのターミナルを用意
まず一つのターミナルで以下を実行

```bash
$ ros2 run mypkg timer
```
起動すると、ターミナルに [INFO] [pomodoro_timer]:mypkg Pomodoro Timer Readyと表示され。待機状態になる。
次に別のターミナルを開き以下を実行

```bash
$ ros2 service call /mypkg/control std_srvs/srv/SetBool "{data: true}"
```
これでタイマーが開始される。
タイマーを一時停止するときは以下を実行

```bash
$ ros2 service call /mypkg/control std_srvs/srv/SetBool "{data: false}"
```
残り時間を確認する場合は以下を実行

```bash
$ ros2 topic echo /mypkg/countdown
```

# Author
* shoma Takatori
* s22C1078PF@s.chibakoudai.jp

# License
- このソフトウェアパッケージは、3条項BSDライセンスの下、再頒布および使用が許可されます。
- @ 2025 shouuuuuu

# Required software
* Ubuntu 22.04
* ROS 2 Humble
* Github Actions(ryuichiueda/ubuntu22.04-ros2:latest)

# Testing enviroment
* ubuntu 20.04 on Windouws

Ⓒ 2025 Shouuuuuu

