# Profiles / Roles

## Profileとは
起動構成のセット。
1つのprofileに、複数のlaunch/nodeを束ねて定義する。

例:
- VR双腕テレオペ
- 俯瞰カメラのみ
- RVizのみ

設定ファイル:
- `vlabor_launch/config/vlabor_profiles.yaml`

起動:
```bash
./scripts/vlabor <profile>
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

## 俯瞰カメラ(C920)のトピック
- node_name: `overhead_camera`
- image topic: `/overhead_camera/image_raw`

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
