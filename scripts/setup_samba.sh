#!/bin/bash
# Samba共有設定スクリプト
# 使用方法: sudo ./setup_samba.sh

set -e

echo "=== VLAbor Samba共有設定 ==="

# 既存の設定を削除
echo "既存の設定を削除中..."
sed -i '/^\[vlabor_datasets\]/,/^$/d' /etc/samba/smb.conf
sed -i '/^\[vlabor_models\]/,/^$/d' /etc/samba/smb.conf

# 新しい設定を追加
echo "新しい設定を追加中..."
cat >> /etc/samba/smb.conf << 'EOF'

[datasets]
   comment = VLAbor Datasets
   path = /home/ros2/ros2_ws/src/vlabor_ros2/datasets
   browseable = yes
   read only = no
   guest ok = yes
   public = yes
   force user = ros2
   force group = ros2
   create mask = 0664
   directory mask = 0775

[models]
   comment = VLAbor Models
   path = /home/ros2/ros2_ws/src/vlabor_ros2/models
   browseable = yes
   read only = no
   guest ok = yes
   public = yes
   force user = ros2
   force group = ros2
   create mask = 0664
   directory mask = 0775
EOF

# 設定確認
echo ""
echo "=== 設定確認 ==="
testparm -s 2>/dev/null | grep -A5 -E "^\[datasets\]|^\[models\]"

# サービス再起動
echo ""
echo "=== Sambaサービス再起動 ==="
systemctl restart smbd

# 状態確認
echo ""
echo "=== 完了 ==="
IP=$(hostname -I | awk '{print $1}')
echo "アクセス方法:"
echo "  Windows: \\\\${IP}\\datasets"
echo "  Windows: \\\\${IP}\\models"
echo "  Mac/Linux: smb://${IP}/datasets"
echo "  Mac/Linux: smb://${IP}/models"
echo ""
echo "パスワードなしでアクセスできます"
