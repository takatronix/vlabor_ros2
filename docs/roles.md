# Roles / Naming

## 目的
物理アーム名と制御の役割を混ぜずに扱うためのルール。

## 基本方針
- 実体名は `left_arm` / `right_arm` に固定。
- 役割ラベル `leader` / `follower` は論理概念としてのみ使う。
- VRがIKを生成する場合でも、実体名は変えない。

## 使い分け例
- VR入力が起点 (source=vr)
  - `left_arm` / `right_arm` は follower
- 物理マスターアームが起点 (source=physical)
  - 物理マスター側が leader
  - もう片側が follower
  - dual構成では `left_arm_2` / `right_arm_2` を leader として扱う

## 推奨パラメータ
```yaml
roles:
  source: "vr"        # vr / physical / recorded
  left_arm: "follower"
  right_arm: "follower"
  left_arm_2: "follower"
  right_arm_2: "follower"

device_map:
  left_arm: "/dev/ttyACM0"
  right_arm: "/dev/ttyACM1"
  left_arm_2: "/dev/ttyACM2"
  right_arm_2: "/dev/ttyACM3"
```

## 注意
`left_follower` のように実体名と役割を混ぜた命名は避ける。
物理デバイスがずれている場合は `device_map` をスワップするだけでよい。
