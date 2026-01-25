# Profiles / Roles

## Profileとは
起動構成のセット。
1つのprofileに、複数のlaunch/nodeを束ねて定義する。

例:
- so101_vr_dual_teleop
- so101_single_teleop
- so101_dual_teleop

singleのleader/follower:
- leader: `right_arm` (/dev/ttyACM1)
- follower: `left_arm` (/dev/ttyACM0)

## Recorderの組み込み
以下のprofileは `lerobot_recorder` を同時起動する:
- so101_vr_dual_teleop (preset: so101_dual_vr)
- so101_single_teleop (preset: so101_single_vr)
- so101_dual_teleop (preset: so101_dual_leader)

設定ファイル:
- `vlabor_launch/config/vlabor_profiles.yaml` (defaults + include list)
- `vlabor_launch/config/profiles/*.yaml` (各プロファイルの定義)

起動:
```bash
./scripts/run <profile>
```

## Roleとは
物理アームの実体名と、制御の役割を分離するための概念。
- 実体名: `left_arm`, `right_arm` など
- 役割: `leader` / `follower`

ルール:
- 実体名は固定、役割だけを切り替える
- 物理デバイスのズレは `device_map` のスワップで吸収

設定ファイル:
- `docs/roles.md`
- `vlabor_launch/config/roles.yaml`

## 設定の置き場所
各プロファイルで使う設定値は `vlabor_launch/config/vlabor_profiles.yaml` に集約する。
`defaults` に共通値、各profileの `variables` に個別の上書きを書く。

## Leader/Followerのコピー (joint_state_mirror_node)
leader側の `joint_states_single` を follower側の `joint_ctrl_single` に転送する。

デフォルトのコピー先:
- single: `/right_arm/joint_states_single` -> `/left_arm/joint_ctrl_single`
- dual: `/left_arm_2/joint_states_single` -> `/left_arm/joint_ctrl_single`
       `/right_arm_2/joint_states_single` -> `/right_arm/joint_ctrl_single`

設定ファイル:
- `vlabor_launch/config/copy_map_single.yaml`
- `vlabor_launch/config/copy_map_dual.yaml`

上書きしたい場合は launch 引数で渡す:
```bash
ros2 launch vlabor_launch vlabor.launch.py profile:=so101_dual_teleop \\
  mirror_dual_config:=/path/to/copy_map.yaml
```

## 俯瞰カメラ(C920)のトピック
- node_name: `overhead_camera`
- image topic: `/overhead_camera/image_raw`
- compressed: `/overhead_camera/image_raw/compressed`

## LeRobot Recorderの設定
LeRobot録画は `lerobot_recorder` のpreset/configを使う。
profileから `episode_recorder.launch.py` をincludeして指定する。

例 (preset):
```yaml
  record_so101_dual:
    actions:
      - type: include
        package: lerobot_recorder
        launch: episode_recorder.launch.py
        args:
          preset: so101_dual_vr
```

例 (config_file):
```yaml
  record_custom:
    actions:
      - type: include
        package: lerobot_recorder
        launch: episode_recorder.launch.py
        args:
          config_file: /path/to/config.yaml
```
