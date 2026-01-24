# vlabor_launch

vlabor 用の起動プロファイルをまとめた launch パッケージ。

## 使い方
```bash
ros2 launch vlabor_launch vlabor.launch.py profile:=vr_teleop_so101
```

簡易コマンド:
```bash
./scripts/vlabor vr_teleop_so101
```

## プロファイル
`config/vlabor_profiles.yaml` に定義されています。

追加プロファイル:
- `overhead_camera` (C920, fv_camera)

## 役割定義
- 役割ルール: `docs/roles.md`
- 役割設定ファイル: `config/roles.yaml`

## Profiles/Role 概要
- `docs/profiles.md`
