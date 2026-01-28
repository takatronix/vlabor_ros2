# VLAbor Mac環境(UTM)セットアップ手順書


VLAbor(ブラボーは)、VLAフレームワークをVRで動作させるためのサーバとアプリシステムの名称です。
Macでのサーバ環境はUTMという仮想環境の上でROS2システムを構築しています。
本手順書はMacでVLAborを起動するための手順書になります。



## UTMのインストール

```
brew install --cask utm
```



## UTMイメージとAPK

最新のUnityアプリ(APK)と、UTMイメージは以下のURLからダウンロードできます。
以下のURLからダウンロード後、zip解凍してUTMアプリで開きます。


https://www.dropbox.com/scl/fo/y94unhv1e3q0bk6q85iib/AJk2_ZJewN4g_-KCO12PDEw?rlkey=dzaamj0thoqervi5n6dee5sdq&st=lyku2u53&dl=0

<img src="/Users/takatronix/Desktop/assets/image-20260128035632184.png" alt="image-20260128035632184" style="zoom:50%;" />



## UTM設定

外部アクセス可能にするために、ネットワーク設定をブリッジ設定にします。

<img src="/Users/takatronix/Desktop/assets/image-20260128073825915.png" alt="image-20260128073825915" style="zoom:50%;" />

ネットワークモードをブリッジに変更します。ブリッジインターフェースはMacのネットワークを指定します。

<img src="/Users/takatronix/Desktop/assets/image-20260128075022908.png" alt="image-20260128075022908" style="zoom:50%;" />



## VLAborサーバー起動


デスクトップに置いてあるvlabor runアイコンをクリックします。

<img src="/Users/takatronix/Desktop/assets/image-20260128075337859.png" alt="image-20260128075337859" style="zoom:50%;" />



### プロファイル選択

VRテレオペをする場合は、VRテレオペを選択しエンターキーを押します。



<img src="/Users/takatronix/Desktop/assets/image-20260128075410499.png" alt="image-20260128075410499" style="zoom:50%;" />



ROS2システムが起動すると以下のような表示になります。VRアプリで、表示されたIPアドレスへ接続してください。
VRアプリのWifiマークが緑になったら接続OKです。



![image-20260128082528286](/Users/takatronix/Desktop/assets/image-20260128082528286.png)



## 共有フォルダ

作成したエピソードデータは、datasetsフォルダに保存され、SMB経由でアクセスできます。
推論用のモデルデータフォルダもSMBでアクセス可能です。





<img src="/Users/takatronix/Desktop/assets/image-20260128084448117.png" alt="image-20260128084448117" style="zoom:50%;" />
