# autoware_feature_environment_recognizer

## 概要

このパッケージは、現在位置がLanelet2のどのlaneletに属しているかを探索し、そのlanelet IDをリストと照合して環境番号を識別する機能を提供します。地図マッチングに重要な特徴がある/ない環境を識別するために使用できます。

## 環境番号の定義

- **環境ID -1**: 無効（laneletが見つからない、またはマップが準備できていない場合）
- **環境ID 0**: 通常環境（デフォルト、どのリストにも該当しない場合）
- **環境ID 1**: 均一形状路（トンネル直線部など均一形状の道路）
- **環境ID 2**: 特徴が少ない道路（地図マッチングに特徴が少ない道路）

## 機能

- Lanelet2マップから現在位置のlaneletを取得
- 設定されたlanelet IDリストと照合
- 環境番号（整数）をパブリッシュ

## ノード

### feature_environment_recognizer_node

現在位置のposeとlanelet mapを受け取り、環境番号をパブリッシュします。

#### サブスクライブ

- `~/input/lanelet2_map` (`autoware_map_msgs::msg::LaneletMapBin`)
  - Lanelet2マップデータ

- `~/input/pose` (`geometry_msgs::msg::PoseStamped`)
  - 現在位置のpose

#### パブリッシュ

- `~/output/environment` (`autoware_feature_environment_recognizer::msg::FeatureEnvironment`)
  - 特徴環境認識結果
  - `header`: 入力位置のタイムスタンプを含むヘッダー
  - `environment_id`: 識別された環境番号
  - `confidence`: 認識結果の信頼度（現在は使用しないが、将来の使用のために予約、デフォルト値: 1.0）

#### パラメータ

- `default_environment_id` (int, default: 0)
  - どのリストにも該当しない場合のデフォルト環境番号（通常環境）

- `search_distance_threshold` (double, default: 10.0)
  - Lanelet検索の距離閾値 [m]

- `search_yaw_threshold` (double, default: 0.785)
  - Lanelet検索のヨー角閾値 [rad] (デフォルトはπ/4)

- `environment_id_<number>.lanelet_ids` (int[], optional)
  - 各環境IDに対応するlanelet IDのリスト
  - `environment_id_1`: 均一形状路（トンネル直線部など）のlanelet IDリスト
  - `environment_id_2`: 特徴が少ない道路のlanelet IDリスト
  - 例: `environment_id_1.lanelet_ids: [100, 101, 102, 103]`

## 使用方法

### Launch

```bash
ros2 launch autoware_feature_environment_recognizer feature_environment_recognizer.launch.xml
```

### パラメータ設定例

`config/feature_environment_recognizer.param.yaml` に以下のように設定します:

```yaml
/**:
  ros__parameters:
    default_environment_id: 0
    search_distance_threshold: 10.0
    search_yaw_threshold: 0.785

    # 環境ID 1: 均一形状路（トンネル直線部など）
    environment_id_1:
      lanelet_ids: [100, 101, 102, 103, 104]

    # 環境ID 2: 特徴が少ない道路
    environment_id_2:
      lanelet_ids: [200, 201, 202, 203, 204]
```

## 実装詳細

このパッケージは以下の機能を使用しています:

- `autoware::experimental::lanelet2_utils::get_road_lanelets_at()`: 位置から道路laneletを取得
- `autoware::experimental::lanelet2_utils::get_closest_lanelet()`: 最も近いlaneletを取得
- `autoware::experimental::lanelet2_utils::get_closest_lanelet_within_constraint()`: 制約付きで最も近いlaneletを取得

## 依存関係

- `autoware_lanelet2_extension`
- `autoware_lanelet2_utils`
- `autoware_map_msgs`
- `geometry_msgs`
- `rclcpp`
- `std_msgs`
