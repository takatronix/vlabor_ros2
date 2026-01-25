# models

学習済みポリシー（ACT, SmolVLA など）の checkpoint を置くディレクトリです。

## 用途

- LeRobot などで学習したモデルの保存先
- 推論時（policy runner）の `checkpoint_path` として参照

## ネットワーク共有

このフォルダは SMB 等で共有可能です。設定は [docs/smb_sharing.md](docs/smb_sharing.md) を参照してください。

## 運用

- 大容量のため、リポジトリでは `models/*` を .gitignore 対象にしています
- 共有ストレージに置いて複数マシンから利用する運用を想定しています
