# joint_state_mirror_node

指定したJointStateトピックを別トピックへコピーして配信するノード。
leader/followerの関係で、leader側のJointStateをfollower側の入力トピックへ流す用途に使う。

## Parameters
- `config_file` (string): copy_mapを定義したYAMLファイルへのパス。
- `mappings` (list): 直接マッピングを渡す場合のリスト。通常は `config_file` を使う。
- `update_stamp` (bool): trueのとき、転送時にstampを現在時刻で上書きする。

## YAML例
`copy_map` を配列で定義する。

```yaml
copy_map:
  - src: /right_arm/joint_states_single
    dst: /left_arm/joint_ctrl_single
```

## 例: launchから起動

```yaml
- type: node
  package: unity_robot_control
  executable: joint_state_mirror_node
  name: joint_state_mirror
  parameters:
    config_file: /path/to/copy_map.yaml
```
