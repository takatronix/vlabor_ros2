# vlabor_launch

vlabor 用の起動プロファイルをまとめた launch パッケージ。

## 使い方
```bash
ros2 launch vlabor_launch vlabor.launch.py profile:=so101_vr_dual_teleop
```

簡易コマンド（vlabor_ros2 ルートで）:
```bash
./scripts/run so101_vr_dual_teleop
```

## プロファイル
`config/vlabor_profiles.yaml` に defaults / include list を置き、実体は `config/profiles/*.yaml` に分割する。

追加プロファイル:
- `overhead_camera` (C920, fv_camera)

## 役割定義
- 役割ルール: `docs/roles.md`
- 役割設定ファイル: `config/roles.yaml`

## Leader/Followerコピー設定
- 単腕用: `config/copy_map_single.yaml`
- 双腕用: `config/copy_map_dual.yaml`

## Profiles/Role 概要
- `docs/profiles.md`
