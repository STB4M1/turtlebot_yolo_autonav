# turtlebot_yolo_autonav

YOLOv8による物体検出とLiDARセンサによる障害物回避を組み合わせた、TurtleBot3の自律ナビゲーション（Physical AI講座 最終課題）

---

## 概要

このプロジェクトは、2025年度「Physical AI講座」の最終課題として制作されたものです。  
Gazebo上に構築したカスタム大部屋環境で、TurtleBot3 (Waffle Pi) が自律的に移動し、YOLOv8による物体検出と、LiDARによる距離推定を用いて、障害物との衝突を回避しながら前進を続けます。

---

## 使用技術・構成

- **ROS2** (Humble)
- **Gazebo** シミュレータ + 自作World (`mixed_room.world`)
- **YOLOv8 (Ultralytics)** による物体検出
- **LiDAR** による障害物回避制御
- 自作ROS2ノード2種：物体検出・障害物回避
- Launchファイルによる一括起動

---

## ディレクトリ構成（例）

```
yolo_nav/
├── launch/
│   ├── full_launch.py           # 環境全体を起動（TurtleBot + World）
│   └── yolo_nav.launch.py       # YOLOノード + 回避ノードを起動
├── src/
│   ├── yolo_node.py
│   └── obstacle_avoidance_node.py
├── models/
│   └── [Gazebo用カスタムモデル群]
├── my_worlds/
│   └── mixed_room.world
```

---

## 実行方法

### 1. ビルド

```bash
colcon build --packages-select yolo_nav
source install/setup.bash
```

---

### 2. GazeboシミュレータとTurtleBot3の起動

```bash
ros2 launch yolo_nav full_launch.py
```

---

### 3. ノード群（YOLO + 障害物回避）の起動

```bash
ros2 launch yolo_nav yolo_nav.launch.py
```

---

### 4. 可視化（任意）

#### `rqt_image_view`

YOLOの検出結果画像（`/yolo/image_annotated`）を確認するには：

```bash
rqt
```

GUI上で `/yolo/image_annotated` トピックを選択してください。

#### `Rviz2`

LiDARスキャン、ロボットの位置、動きなどを統合的に確認するには：

```bash
rviz2
```

RViz内で以下のトピックを表示すると便利です：
- `/scan`（LiDARスキャン）
- `/tf`（ロボット座標系）
- `/cmd_vel`（速度コマンド）

---

## 必要パッケージ

- ROS2 (`rclpy`, `sensor_msgs`, `geometry_msgs`)
- `cv_bridge`
- `ultralytics`

```bash
pip install ultralytics
```

---

## トピック構成

| ノード名                  | 購読トピック            | 配信トピック               |
|---------------------------|--------------------------|-----------------------------|
| `yolo_node.py`            | `/camera/image_raw`      | `/yolo/image_annotated`     |
| `obstacle_avoidance_node.py` | `/scan`               | `/cmd_vel`                  |

---

## 工夫ポイント

- 自作ワールドに多様な障害物・構造物を配置し、検出・回避の挙動を観察できるように設計
- YOLOv8の軽量モデルを使用し、推論のスピードと精度のバランスを確保
- `full_launch.py` により再現性の高い環境起動を実現

---

## 補足事項

`my_worlds/models` フォルダに含まれるカスタムモデルをGazeboで正しく読み込ませるには、以下のようにしてホームディレクトリ下の Gazebo モデルフォルダにコピーしてください：

```bash
cp -r my_worlds/models/* ~/.gazebo/models/
```

この操作を行うことで、`mixed_room.world` 内で使用されるカスタムモデルがGazebo上で正しく表示されます。
