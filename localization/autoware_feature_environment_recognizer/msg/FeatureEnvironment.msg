# Feature environment recognition result
# 特徴環境認識結果

# Header with timestamp from input pose
# 入力位置のタイムスタンプを含むヘッダー
std_msgs/Header header

# Environment ID
# -1: Invalid (lanelet not found or map not ready)
# 0: Normal environment (default)
# 1: Uniform road (e.g., tunnel straight sections, uniform shape roads)
# 2: Feature-poor road (roads with few features for map matching)
# 環境番号
# -1: 無効（laneletが見つからない、またはマップが準備できていない）
# 0: 通常環境（デフォルト）
# 1: 均一形状路（トンネル直線部など）
# 2: 特徴が少ない道路
int32 environment_id

# Confidence of the recognition result (0.0 to 1.0)
# Currently not used, reserved for future use
# 認識結果の信頼度（0.0から1.0）
# 現在は使用しないが、将来の使用のために予約
float64 confidence
