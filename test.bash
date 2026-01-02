#!/bin/bash
# SPDX-FileCopyrightText: 2025 Shoma Takatori
# SPDX-License-Identifier: BSD-3-Clause

ng () {
    echo "${1}行目が違うよ"
    res=1
}

res=0

# ビルドと環境設定
[ -d build ] || colcon build
source install/setup.bash

### NORMAL INPUT ###
# 出力されている「Pomodoro」という単語が含まれているかチェック
# ログ出力は標準エラー(stderr)に出ることがあるため、2>&1 でまとめます
out=$(timeout 5s ros2 run mypkg timer 2>&1 | grep "Pomodoro" || true)

if [ "${out}" = "" ]; then
    ng "$LINENO"
fi

### NODE LIST CHECK ###
# ノードをバックグラウンドで起動
ros2 run mypkg timer > /dev/null 2>&1 &
node_pid=$!

# 起動を待つ時間を少し長めに（2秒から5秒へ）
sleep 5

# ノード名のチェック（出力されたログに合わせて /pomodoro_timer を探す）
ros2 node list | grep -q "/pomodoro_timer" || ng "$LINENO"

# 終了処理
kill $node_pid

[ "${res}" = 0 ] && echo OK
exit $res
