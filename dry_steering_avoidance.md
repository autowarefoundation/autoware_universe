# dry_steering_avoidance_moduleの実装手順

autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_moduleを参考に新しい dry_steering_avoidance_moduleを作成してください。

autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/README.md にはstart plannerのロジックの説明があります。

Geometric pull out のみ steering avoidance moduleとして実装したいです。

# ステップ1

start_planner_module から Geometric pull out  のみを残したモジュールとして、steering_avoidance_moduleを作成してください。

CMakeLists.txtやpackaeg.xmlやconfigファイルなど関連する箇所をよく調べて追加してください。

geometric_pull_out.hppはincludeしてライブラリとして使ってください。

モジュールの発動条件には、
- egoの前に物体が存在する
- egoが停止している (kinematic stateから取得)
上記のand条件が一定時間(パラメタ化する。デフォルト秒)満たすことにしたい。

発動条件は `isExecutionRequested()` で書くことができます。


経路計画には `GeometricPullOut::plan()` を使ってください。
start_poseは現在のegoの位置 (kinematic stateから取得)です。
goal_poseは、egoの前、物体の右 (パラメタで指定。デフォルト右)の位置にしたいです。
goal poseは複数のposeを順にforで与えたいです。縦方向の位置と横方向の位置で経路のサンプリングをしてください。
