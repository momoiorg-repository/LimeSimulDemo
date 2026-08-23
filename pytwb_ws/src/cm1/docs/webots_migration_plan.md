# Gazebo → Webots 移行計画

対象: `~/bin/run_all` および関連スクリプト（`run_any_world`, `run_all_with_anyworld`, `run_base`,
`run_empty`, `run_turtle_base` 等）で起動しているシミュレーション環境一式。

## 1. 現状の依存関係サマリ（調査結果）

### 起動フロー（run_all）
```
run_all
 ├─ turtlebot3_lime_bringup gazebo.launch.py   … Gazebo本体 + ロボットspawn
 ├─ turtlebot3_lime_moveit_config servo.launch.py (use_sim:=true)
 └─ turtlebot3_lime_bringup moveit_navigation_use_sim_time.launch.py (map: sim_house_map.yaml)
```
実体パッケージ: `~/turtlebot3_ws/turtlebot3_lime/`（build済みで `install/` に配置、`src/`は空）

### Gazebo依存箇所
| 箇所 | 内容 |
|---|---|
| `turtlebot3_lime_description/urdf/turtlebot3_lime.urdf.xacro` | `libgazebo_ros2_control.so` プラグイン（ros2_control接続） |
| `turtlebot3_lime_description/ros2_control/turtlebot3_lime_system.ros2_control.xacro` | `use_sim=true`時 `gazebo_ros2_control/GazeboSystem` ハードウェアプラグイン |
| `turtlebot3_lime_description/gazebo/turtlebot3_bigwheel_orin.gazebo.xacro` | `libgazebo_ros_diff_drive.so`（差動駆動）, `libgazebo_ros_ray_sensor.so`（LiDAR）, `librealsense_gazebo_plugin.so`（RealSenseカメラ: color/depth/infra1/infra2/pointcloud） |
| `turtlebot3_lime_description/gazebo/open_manipulator_sara.gazebo.xacro` | アーム各リンクの摩擦・接触パラメータ、simple transmission |
| `turtlebot3_lime_bringup/launch/gazebo.launch.py` | `gazebo_ros`起動 + `spawn_entity.py`でロボット投入 |
| `turtlebot3_lime_bringup/worlds/*.model`, `/project/resource/sim_house.world` | Gazebo SDF形式のワールド（自作の家レイアウト） |
| `/project/resource/model_editor_models/*` | 独自SDFモデル（aruco_box0〜9, coke_can, orange_ball, euro_pallet, Coke） |
| `/project/resource/map/sim_house_map.yaml` | 上記Gazeboワールドに対してSLAMで生成したNav2用地図 |
| `/project/lib_ws/src/realsense`（`realsense_gazebo_plugin`） | RealSenseのGazebo専用プラグイン実装 |
| `/project/lib_ws/src/IFRA_LinkAttacher`（`ros2_linkattacher`） | Gazebo上でリンク同士を仮想接続する「把持」シミュレーション用プラグイン＋ROS2サービス |
| cm1本体 `lib/actor/system.py` | `gazebo_msgs.msg.ModelStates / LinkStates` を直接購読 |
| cm1本体 `lib/actor/tools.py` | `/ATTACHLINK`, `/DETACHLINK` サービス（linkattacher_msgs）を呼び出し、`link7`と対象物体をアタッチ/デタッチ（ピッキング演出） |
| `/project/lib_ws/src/ros_actor`, `actor_interface` | 要調査。Gazebo Actor（歩行者アニメーション）機能に依存している可能性あり |

### 環境
- ROS 2 Humble。`ros-humble-webots-ros2*` は apt で提供されているが未インストール（`webots-ros2-driver`, `webots-ros2-control`, `webots-ros2-importer` 等）。
- Webots本体も未インストール。
- `DISPLAY=:0` が設定済みでGUI表示は可能な模様（要確認）。

## 2. 移行方針の骨子

- URDF/xacroはWebots専用に作り直さず、既存の `turtlebot3_lime.urdf.xacro` を `webots_ros2_importer`（URDF→Webots PROTO変換ツール）にかけて流用する。
- センサー・アクチュエータ駆動は `webots_ros2_driver` プラグイン方式（`<devices>`定義 + Pythonドライバ or 標準プラグイン）に置き換える。
- `ros2_control`のシム側ハードウェアは `gazebo_ros2_control/GazeboSystem` → `webots_ros2_control` の `WebotsSystem` 相当に置き換える（既存のコントローラ設定ymlは概ね流用できる想定）。
- ワールド（`sim_house.world`）と独自オブジェクト（aruco box等）はWebots `.wbt`/PROTOとして作り直す（自動変換ツールなし、手動再構築が必要）。
- 把持シミュレーション（LinkAttacher）とアクター（歩行者）機能はGazebo固有プラグインに依存しているため、Webots Supervisor APIベースで個別に再実装する。

## 3. マイルストーン

### M0. 事前調査・環境準備 ✅ 完了
- Webots R2025a + `ros-humble-webots-ros2*` 一式インストール済み（ユーザー側で実施）
- `webots_ros2_turtlebot`サンプルで動作確認済み
- `ros_actor` / `actor_interface` の精査はM6着手時に実施予定（未着手）

### M1. ロボットモデル移行（URDF→Webots）✅ 完了
- `turtlebot3_lime.urdf.xacro`を`webots_ros2_importer`（xacro2proto）でPROTO化し、`~/webots_ws/src/turtlebot3_lime_webots`パッケージとして新規作成
- 発生した重大な物理不安定（起動直後に関節が暴走・破綻する不具合）を特定・修正:
  - 原因1: アーム各リンク(link1〜7・グリッパ)の当たり判定に、mm→m変換されていない生STLメッシュがそのまま使われ、1000倍大きい凹形状として干渉していた → ボックス近似に置き換えて解消
  - 原因2: IMUの`Accelerometer`/`Gyro`が物理ボディを持つ親を持たず機能していなかった → 専用`Solid`（微小質量）でラップして解消
  - 原因3: `webots_ros2_driver::Ros2IMU`プラグインのパラメータ名は`enabled`ではなく`alwaysOn`が正しかった
  - 副次修正: ロボットのスポーン高さが地面から浮いていたため起動直後に不自然な着地動作が発生 → 正しい接地高さに修正
  - **原因4（最重要・M4検証時に発覚）: `boundingObject`/`physics`が`Robot`ノード自身ではなく、その中の入れ子の`Solid`（`base_link`）に付いていた**ため、Webotsがロボット全体を1つの動く剛体として認識できず、重力すら効かず、車輪トルクも一切伝わらず完全に静止したままだった（`/cmd_vel`を送っても車輪だけ空転し、車体は1mmも動かない）。摩擦係数やキャスター形状をいくら調整しても直らなかった真因がこれ。公式の`TurtleBot3Burger.proto`と構造を比較して発見。`boundingObject`/`physics`を`Robot`ノード直下のフィールドに付け替え、内部の子ノード群は`Transform`（元のZオフセット0.06を維持）でラップし直すことで解消。GPSデバイス（ホイールオドメトリに頼らない検証用）を追加し、実際に並進・回転することを確認済み
  - **原因5（走行時の左右ドリフト）: 元のURDF自体に左右非対称なバグがあった。** `turtlebot3_bigwheel_orin.urdf.xacro`で`wheel_left_joint`にのみ`<dynamics damping="0.7" friction="1.0"/>`が設定され、`wheel_right_joint`には`<dynamics>`タグ自体が無かった（右は暗黙的に0扱い）。Gazebo版はSDFレベルの`libgazebo_ros_diff_drive.so`が実際の駆動を担っており、URDFのjoint dynamicsを経由しなかったため問題が潜在化していたが、Webots版はros2_control経由でこの値をそのまま使うため、速度に比例して左右の抵抗差が増幅され、高速時ほど大きく左に曲がる不具合として顕在化した（低速0.15m/sではほぼ無視できる量だが、0.4m/sでは5秒で約57°も回転）。`wheel_right_joint`にも同じ`<dynamics>`を追加して解消。**ソース側のURDF xacroに修正済み**（Gazebo版・実機版にも影響する可能性があるため今後要注意）
- `/cmd_vel`での走行（GPS実座標で確認、高速0.4m/sでも直進性を確認）、回転（IMU姿勢で確認）、アームへの目標姿勢送信（`FollowJointTrajectory`）、静止状態でのIMU（重力加速度9.81を正しく検出）まで動作確認済み

### M2. ros2_control連携 ✅ 完了
- `turtlebot3_lime_system.ros2_control.xacro`に`use_webots`分岐を追加し、`webots_ros2_control::Ros2ControlSystem`プラグインを設定（**ソース側に正式追加**。以後`xacro ... use_webots:=true`で再現性ある生成が可能）
- Webots版コントローラ設定`config/ros2_control.yml`は実機用`hardware_controller_manager.yaml`をベースに作成し、完全整合を確認（`diff_drive_controller`をros2_control経由で使う点も実機版に合わせた。Gazebo版はSDFレベルの専用プラグインで駆動していたため、これは意図的な設計変更）
- 差異: `imu_broadcaster`はWebots版では使わず、IMUはM3のWebots専用プラグイン経由に統一（`webots_ros2_control`がIMU等のセンサー型state interfaceに未対応のため）

### M3. センサー移行 ✅ 完了（LiDAR/カメラ/IMU）
- LiDAR: `Lidar`ノードで`/scan`を配信、既存仕様（360サンプル, 0.12〜3.5m, 5Hz）を維持
- RealSenseカメラ: `Camera`/`RangeFinder`ノードで color/depth/infra1/infra2を構成。`topicName`+`imageSuffix`/`cameraInfoSuffix`の組み合わせで、`lib/actor/system.py`が購読する`/camera/camera/color/image_raw`等の**トピック名を完全一致**させることを確認
- IMU: `InertialUnit`+`Gyro`+`Accelerometer`を追加し、`/imu`で正しい値を配信することを確認
- 新規xacro `turtlebot3_lime_description/webots/turtlebot3_lime.webots.xacro`としてソースに正式追加（`<gazebo>`タグと同様の役割）
- 未確認: 画像のエンコーディング（現状`bgra8`）が実機/Gazebo版の想定と一致するかは未検証（M9統合テストで確認予定）
- **追記（M7検証時に発覚・修正）: 独自実装の`Lidar`ノードが、Webots上では`/scan`トピック自体は正常に配信され続けるのに、`ranges`が常に全方向`inf`（未検知）になる不具合があった。** 壁のcollision・GPU/レンダリング設定・updateRate・fieldOfViewなど考えられる原因を広く検証したが特定できず（壁への物理接触自体は正常、公式サンプル`webots_ros2_turtlebot`は同一環境で正常動作、というところまで切り分けたが、独自`Lidar`ノードが機能しない根本原因は不明のまま）。最終的に、実機TurtleBot3が使う本物のLDS-01センサーの公式PROTO（`https://raw.githubusercontent.com/cyberbotics/webots/develop/projects/devices/robotis/protos/RobotisLds01.proto`、EXTERNPROTO経由）に丸ごと差し替えて解消。**注意: この方式はWebots起動時にGitHubからのフェッチ（初回のみ、以降はローカルキャッシュ`~/.cache/Cyberbotics/Webots/assets/`）が必要になるため、インターネット接続に依存する形になった。**

### M4. ワールド・オブジェクト移行 ✅ 完了
- `sim_house.world`のSDFを解析（`<state>`ブロックから各壁の絶対座標を抽出）し、壁30枚を全て箱形状として`sim_house.wbt`に自動生成。手作業での間取り再構築は不要だった（家は単純な直方体の壁の集合だったため）
- `model_editor_models`配下の全13オブジェクトをPROTO化: `Coke`, `coke_can`, `orange_ball`, `euro_pallet`, `aruco_box0`〜`9`
  - `behavior/simvison.py`の`LookForCoke`/`CheckCoke`（`bt_search.xml`で使用）が実際に依存しているのは`Coke`オブジェクトと判明。カメラ画像ベースの検出のため、M3のカメラトピック整合が効いてくる
  - メッシュ(DAE/OBJ)+テクスチャ資産を`protos/objects/`配下にコピーし、Gazebo専用の`model://`テクスチャURIを相対パスに修正
  - **当たり判定はM1の教訓を活かし、全オブジェクトで単純プリミティブ（Box/Cylinder/Sphere）に置き換え**、生メッシュを直接boundingObjectに使わないようにした（元のSDFでもaruco_boxは箱コリジョンだったが、coke_can/Cokeは生メッシュコリジョンだったため要注意だった）
  - `euro_pallet`は参照先メッシュ(`pallet.dae`)がプロジェクト内に実在しなかった（Gazeboのオンラインモデルデータベース依存だった模様）ため、標準的なユーロパレット寸法(1.2×0.8×0.144m)の木箱として作成
- ロボット初期位置は`translation 0 0 0.002`でワールドファイルに直接配置（M1のスポーン高さ修正と同じ考え方）
- 全オブジェクト配置済みの`sim_house.wbt`で起動・安定動作を確認済み（衝突暴走なし）

### M5. 把持（LinkAttacher）機能の対応 ✅ 完了
- **Webots標準の物理演算（グリッパのリンク間摩擦・接触）のみで把持動作が成立することを確認。LinkAttacher相当の代替実装は不要と判断。**
- 検証の過程で発覚・修正した不具合:
  - **原因1: `gripper_left_link`/`gripper_right_link`のboundingObjectに、STLメッシュ全体のbboxをそのまま使ったBoxを採用していた**ため、指メッシュがほとんど届いていない高さ（ローカルZ全域）でも、実際は局所的な「くちばし状の爪」形状（ローカルZ=0〜5mmのみ）がY=-9〜-12mmまで深く食い込む値を、指全体に均一適用してしまっていた。結果、フルオープンでも当たり判定上は常に対象物とわずかに重なった状態になり、「見た目は隙間があるのに閉じられない／開ききれない」不具合が発生。STLメッシュの頂点分布をZ帯ごとに解析し、指本体（浅い当たり判定）とくちばし部分（狭い高さ・深い当たり判定）の**2つのBoxに分割**することで、実メッシュの形状に近づけて解消
  - 原因2: `gripper_left_link`/`gripper_right_link`/`end_effector_link`の質量が0.001kg(1g)と極端に軽く、対象物（コーク缶0.03kg）との質量比が大きすぎたため、把持中の振動でジョイントが可動域（`minPosition`/`maxPosition`）を大きく超えて振り切れ、指が「外れる」ような物理破綻が発生。質量を0.02kgに引き上げて解消
  - 原因3: グリッパー`LinearMotor`の`maxForce`(1.0N)がジョイント自体の`staticFriction`(1.0)とほぼ同値で、モーターが自身の関節摩擦にすら安定して勝てず、open動作が中途半端な位置でstallしていた。`maxForce`を2.0Nに引き上げて解消
- 追加修正（原因4）: くちばし部分のboundingObjectがY=-9.15mm相当まで深く食い込んでいたため、フルオープン時のクリアランスが約0.85mmしかなく、接触判定の丸め誤差で「離せる時と離せない時がある」という間欠的な不具合が発生。くちばしの食い込みをY=-7mm相当まで浅くし、クリアランスを約3mmまで拡大して解消（握力はモーターが缶表面でstallすることで確保されるため、浅くしても把持力への影響は無い）
- 上記修正後、掴む→持ち上げる→移動→リリースの一連の動作を複数回繰り返しても安定して成立することをユーザーが実機（Webots GUI）で確認済み

### M6. アクター（`lib/actor/*`）機能の対応 ✅ 完了
- cm1本体`lib/actor/tools.py`の`/ATTACHLINK`, `/DETACHLINK`（LinkAttacher関連）呼び出しは、M5でWebots標準物理演算のみで把持が成立することが確認できたため**不要と判断し、ユーザーが削除**。従来通りの使い方（`lib/actor/tools.py`のAPI）は変更なく動作することを確認済み
- 他にGazebo依存は無いことを確認（`ros_actor`/`actor_interface`のGazebo Actor歩行者アニメーション依存は当初の懸念だったが、実質的な依存はLinkAttacher関連のみだった）
- 残課題（軽微・M10でまとめて対応予定）: `lib/actor/system.py`に`gazebo_msgs.msg.ModelStates, LinkStates`のimportと`/model_states`/`/link_states`の購読登録が残っているが、コードベース内で実際にこのデータを読んでいる箇所は無く、未使用のデッドコード。実行への影響は無い

### M7. ナビゲーション・マップ再検証 ✅ 完了
- 新Webotsワールド上でSLAM（cartographer, `run_mapping`）を再実行し、地図を再生成 → `sim_house_webots`として保存済み
- Nav2（`moveit_navigation_use_sim_time.launch.py`）が新地図・新オドメトリで正常動作することを確認済み

### M8. 起動スクリプト・launchファイルの置き換え ✅ 完了
- 調査の結果、`servo.launch.py`・`moveit_navigation_use_sim_time.launch.py`・`navigation2*.launch.py`はいずれもGazebo非依存（純粋にMoveIt/Nav2のみに依存）と判明。**新規launchファイルの作成は不要**で、`gazebo.launch.py`だけを`turtlebot3_lime_webots`パッケージの`robot_launch.py`に置き換えれば済んだ
- `~/bin/run_all`, `run_any_world`, `run_all_with_anyworld`, `run_base`, `run_nav`を更新（`webots_ws/install/setup.bash`のsource追加、`gazebo.launch.py`→`robot_launch.py`、マップ参照を`sim_house_map.yaml`→`sim_house_webots.yaml`に変更）
- `run_empty`（Gazebo空ワールド）・`run_turtle_base`（Gazebo公式デモワールド）は、対応する専用Webotsワールドが無いため`turtlebot3_lime_test.wbt`（ロボットのみの簡易検証用ワールド）に暫定対応
- `run_rviz`は元々参照先の`moveit_gazebo2.launch.py`が既に存在せず、今回の移行以前から壊れていたスクリプトと判明。対応スコープ外として未着手のまま維持
- `run_fake`は`turtlebot3_manipulation_moveit_config`という別パッケージを使っており、今回の移行と無関係なため未着手
- `run_mapping`, `run_map_save`, `run_teleop`, `run_navigation`, `run_servo`はGazebo非依存のため無改修で流用可能と確認済み（前者2つはM7で実際に使用・動作確認済み）
- 新しいマップ(`sim_house_webots.pgm`/`.yaml`)を`/project/resource/map/`に配置（旧`sim_house_map.*`は削除せず並存。削除はM10で判断）
- 既存Gazebo版launch/worldファイル自体はまだ削除していない（M10でまとめて削除予定）

### M9. 統合テスト・回帰確認（進行中、`bt_catch`は複数回成功を確認）
- cm1の行動ツリー（`behavior/`, `trees/`）を実行し、以下の不具合を発見・修正:
  - **カメラの向きが完全に間違っていた**: `webots_ros2_importer`が生成した4カメラ（color/depth/infra1/infra2）の`rotation`値が、標準的なROS光学フレーム変換の**逆行列**になっており、前方ではなく横方向を向いていた。実測（Webotsの向きは左右反転済みのローカル座標系）で正しい値を導出し修正（最終的に回転なし=identityが正解だった）
  - **深度カメラの単位不一致**: Webotsの`RangeFinder`は32bit浮動小数点・メートル単位・`inf`=未検知で配信するが、`lib/actor/cognitive.py`は実機RealSense標準の16bit整数・ミリメートル単位・`0`=未検知を前提に書かれていた。`normalize_depth_mm()`ヘルパーを追加し、読み込み直後に変換することで対処
  - **`argmin()`が無効値(0)を誤検知**: 深度データの「一番近い点」を探す処理が無効ピクセルを無視しておらず、未検知領域があると誤って最短距離と判定していた。`argmin_valid`/`argmin_valid_band`（複数行の帯を探索）で解消
  - **`measure_center2`が固定行のみ・色情報を無視**: 決め打ちの画像行のみを見ており、対象の位置に追従していなかった。さらに、掴む直前にアームがカメラ視界を横切ると、色を無視して「その行で一番近いもの」（＝アーム自身）を対象と誤認識していた。`pic_find`の色検出結果に追従する行選択＋幅スキャンへの色マスク適用で解消
  - **色検出のHSVしきい値がGazebo基準だった**: `practice_ws/images/my_dtcs.py`の`d_coke`関数の彩度しきい値(`S:230-250`)が、Webots上の実際のレンダリング結果（実測`S:36-236`）とほぼ合っておらず、検出できてもごく少数ピクセルしかヒットしない不安定な状態だった。実測分布に基づき`S:150-255`に緩和
  - **`RangeFinder`の`maxRange`が未設定でデフォルト1.0mのまま**だった（実機は最大10m程度）。5.0mに変更
  - **`RangeFinder`/`Camera`の`noise`が過大**（特に深度は相対値0.1=1mで±10cm相当）だった。シミュレーション用途では不要と判断し全カメラで0に変更
  - **`targetted_walk`系の制御ループにレート制限が無く**、カメラの更新頻度を超える速度で色+深度の全処理を繰り返しており、CPU負荷とPID制御の不安定化の一因になっていた。50ms間隔の制限を追加
  - **`reach_coke`/`shift`/`approach`が、移動距離が小さいときに`atan2(dy,dx)`でノイズだらけの向きを計算**し、ロボットが変な方向を向く不具合があった。移動量が5cm未満なら現在の実向き（TFのquaternionから算出）を使うフォールバックを追加
  - **グリッパーの閉じる衝撃で対象物が暴れる**: `maxVelocity`(4.8→1.0)と`controlPID`のP項(10→6)を下げて閉じる勢いを緩和
  - `Pick`→`ArmHome`の間に、掴んだ物を地面にこすらず確実にクリアランスを取ってから収納する`pick_up`アクター（`manipulator.py`）・`PickUp`ビヘイビア（`manipulation.py`）を新規追加し、`bt_catch.xml`に組み込み
  - **セッション中に片付け忘れた孤立プロセス**（`move_group`の重複等）が`/move_action`アクションサーバーの重複警告とシステム全体の速度低下を引き起こしていたことが判明。あわせて、`moveit_navigation_use_sim_time.launch.py`が起動する2つのRViz（MoveIt用+Nav2用）が非常に重く（各プロセスでCPU使用率80〜100%）、AMCLの`map→odom`TF更新頻度を著しく低下させ、ビジョンベースの位置推定誤差の一因になっていた。片方を閉じることで`/clock`の更新頻度が約3Hz→19Hzまで改善することを確認
- 上記修正により、`bt_catch`（コーク缶の視覚探索→接近→把持→収納）が複数回にわたり成功することを確認。引き続き成功率の詰めを実施中
- `mig_test.py`は実際には実行可能なテストスクリプトではなく、`@actor`関数から自動生成されたクラス定義の参照ファイル（import先が存在しないプレースホルダ）と判明。回帰テストとしての流用は不可

### M10. 後片付け
- Gazebo専用パッケージ（`realsense_gazebo_plugin`, 旧`ros2_linkattacher`実装等）の要否を確認し、不要なら削除
- ドキュメント更新（README等があれば）

## 4. 決定事項（2026-08-19時点）
- Gazebo関連コード・launchファイルは移行完了後に**完全削除し一本化**する（M10で対応）
- `sim_house_map.yaml`はWebotsワールドで**mapping（cartographer）を取り直して再生成**する（M7）
- 把持（LinkAttacher）機能は、**M5でWebots標準の物理演算のみで安定動作することを確認済み**。代替実装（Supervisor経由のLinkAttacher相当）は不要と決定（2026-08-20）
- アクター（歩行者）機能のスコープは、M6着手時に`ros_actor`/`actor_interface`の依存を洗い出してから決定する
- Webots本体・`ros-humble-webots-ros2*`のインストールは**ユーザー側で実施**。Claude側の作業はインストール完了後（M0のインストール以外の部分）から着手する
