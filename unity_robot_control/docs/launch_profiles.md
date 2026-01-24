# Launch Profiles

`config/launch_profiles.yaml` に起動構成を定義します。

## 使い方
```bash
ros2 launch unity_robot_control teleop_profiles.launch.py \
  profile:=vr_dual_arm_teleop
```

## 追加・編集
- `profiles.<name>.actions` に `node` / `include` / `execute` を追加。
- `enabled` を `true/false` か `${launch_arg}` で切り替え。
- `args` の `${var}` は launch 引数から解決。
- `omit_empty_args` に指定した引数は空なら渡されません。

## 例
```yaml
profiles:
  example:
    actions:
      - type: node
        package: unity_robot_control
        executable: vr_dual_arm_control_node
```
