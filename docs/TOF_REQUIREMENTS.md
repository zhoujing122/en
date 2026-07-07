# 3-ToF Requirements

本文档用于对齐 `robot_slamd` 第一版 3 路红外 ToF 接入需求、数据格式、安装外参、算法使用边界和验收标准。

传感器误差预算和标定验收范围见 `docs/SENSOR_ERROR_BUDGET.md`。本文档中的 ToF 要求必须与该误差预算一起执行。

当前阶段目标不是把 ToF 直接用于 pose 写回，而是先完成：

```text
3 路 ToF 稳定采集
ToF 日志可观测
ToF confidence/status 进入建图
ToF 异常不污染地图
ToF/map residual 诊断可复现
```

## 1. 硬件配置

第一版使用 3 路独立红外 ToF：

```text
front: 前向 ToF
left:  左侧 ToF
right: 右侧 ToF
```

每一路 ToF 需要明确：

```text
安装位置 x_m
安装位置 y_m
安装角 yaw_deg
最小有效距离 min_range_m
最大有效距离 max_range_m
是否有遮挡
是否可能互相串扰
线束连接方式
板端设备节点或 ioctl 通道
```

坐标约定：

```text
base_link 原点：机器人底盘中心
x 正方向：机器人前方
y 正方向：机器人左方
yaw 正方向：逆时针
```

建议初始外参：

```text
front yaw_deg = 0
left  yaw_deg = 90
right yaw_deg = -90
```

实际量产前必须重新测量外参，不能长期依赖默认值。

当前供应商确认的 ToF 规格：

```text
stable distance output: supported
measurement range: 0.02-5.00 m
blind zone: 0.02 m
frame rate: 30 Hz
distance unit on wire: mm
invalid measurement: confidence = 0
low-confidence warning: confidence < 70 is not recommended for mapping
status bits: not provided by this ToF module
three routes: independent measurement/output
crosstalk condition: avoid overlapping optical spots on the same target
```

## 2. 板端数据接口

每路原始数据至少需要提供：

```text
timestamp_us
sensor_id
range_mm
range_status
confidence
read_ok
```

字段含义：

```text
timestamp_us:
  板端采样时间，单位 us。优先使用请求式时间戳：主控发起请求时带 timestamp_us，
  ToF 返回距离时原样带回该 timestamp_us。

sensor_id:
  front / left / right。

range_mm:
  距离值，单位 mm。进入 `robot_slamd` 后再转换成 m。

range_status:
  可选状态位。MT3801 等带状态位的设备可以填真实状态。
  无状态位 ToF 必须由驱动层合成 `range_status=0`，并用 confidence 表示有效性。

confidence:
  置信度，范围建议统一为 0~100。`confidence=0` 表示 invalid/no measurement。

read_ok:
  ioctl 或底层读取是否成功。
```

无状态位、请求式 ToF 的驱动契约：

```text
host request:
  timestamp_us: monotonic board time in us, captured immediately before request send

device response:
  echo timestamp_us unchanged
  range_mm in mm
  confidence in 0..100

driver mapping to robot_slamd:
  confidence == 0      -> range_status = 0, rejected by invalid_confidence
  0 < confidence < 70 -> range_status = 0, rejected by rejected_confidence
  confidence >= 70    -> range_status = 0, valid candidate after range/filter checks
```

不要把 `confidence=0` 合成为 `range_status=255`。`range_status=255` 在现有算法中表示
`free_only/no-object`，会更新 free space；无状态位 ToF 无法可靠区分 no-object 和无效值时，
应保持不更新地图。

当前代码侧期望：

```text
tof.source = mt3801_ioctl
```

真实硬件 ToF 模式下不允许 silent fallback 到 sim。

也就是说：

```text
设备打不开 -> fatal 或明确 error
初始化任一步失败 -> stop/power_off/close 后 fatal
ioctl 失败 -> tof_log.csv 记录 tof_read_fail
连续空读 -> tof_log.csv 记录 tof_gap
某一路缺失 -> tof_log.csv 记录 tof_missing_front / tof_missing_left / tof_missing_right
```

## 3. ToF 数据质量要求

每一路 ToF 需要独立统计健康度：

```text
hit_rate
reject_rate
gap_count
confidence_mean
read_fail_count
missing_count
```

异常标签建议：

```text
tof_read_fail
tof_gap
tof_missing_front
tof_missing_left
tof_missing_right
tof_low_confidence
tof_invalid_status
tof_jump
tof_unhealthy
```

如果某一路长期异常：

```text
暂停该路参与建图
tof_log.csv 写明原因
metrics.csv 记录统计
```

第一版宁可少更新地图，也不能让坏 ToF 数据污染地图。

每一路 ToF 的测距误差验收范围：

```text
0.02-0.50 m: absolute error <= 0.03 m
0.50-5.00 m: absolute error <= max(0.05 m, measured_range * 0.03)
confidence >= 70 for mapping hits
front/left/right timestamp skew target <= 20 ms, maximum <= 50 ms while moving
```

超过上述范围时，该路 ToF 只能用于诊断日志，不能作为建图合格依据。

## 4. ToF 在建图中的使用

ToF 不直接生成 pose。当前主链路是：

```text
wheel encoder + IMU gyro_z -> 2D odom pose
3 ToF -> occupancy grid map update
```

每一路 ToF 作为一条 ray 使用：

```text
ray 中间区域 -> free
有效 hit 位置 -> occupied
no-object status, if the device really provides it -> free only
invalid/status 异常 -> 不更新 occupied
confidence 低 -> 降低 log-odds 更新强度
```

confidence 加权建议：

```text
occ_delta = log_odds_occ * confidence / 100.0
```

如果 confidence 低于阈值：

```text
不作为 occupied hit
记录 rejected_confidence 或 invalid_confidence
不参与地图 occupied 更新
```

第一版有效置信度阈值：

```text
confidence == 0       -> invalid/no measurement，不参与建图 hit
0 < confidence < 70  -> low confidence，只记录日志，不参与建图 hit
confidence >= 70     -> valid candidate，可进入 ToF filter 和地图更新
```

## 5. 地图污染防护

ToF 建图必须受 odom quality 和 ToF health gate 约束。

以下情况暂停 ToF map update：

```text
encoder_jump
encoder_gap
gyro_spike
imu_gap
fast_linear
fast_angular
low_odom_quality
tof_unhealthy
```

当 odom_quality < 0.5：

```text
暂停 ToF map update
tof_log decision 写 mapping_skipped_low_odom_quality
metrics.csv 记录 low_odom_quality_pauses
```

这样做的目的：

```text
定位质量差时，不用错误 pose 把 ToF hit/free 写进地图。
```

## 6. ToF Pose Correction 边界

当前阶段 ToF 不写回 Localizer。

允许的模式：

```text
tof_pose_correction.mode = score_only
tof_pose_correction.mode = yaw_candidate
```

禁止误用：

```text
yaw_only -> fatal
xy_yaw   -> fatal
```

当前 observe-only 诊断输出：

```text
tof_correction_log.csv
candidate_yaw.csv
yaw_residual_curve.csv
yaw_residual_summary.csv
```

`yaw_candidate` 只做：

```text
固定 x/y
只搜索 yaw
输出 candidate yaw
输出 residual / margin / reliability
不修改 pose
不修改 yaw_enc_
不修改 yaw_imu_
不影响 trajectory.csv
不影响 mapping 使用的 pose
```

进入 yaw 写回之前，必须先用真实 ToF 数据证明：

```text
candidate yaw 连续稳定
best residual 明显优于 odom residual
best_second_margin 足够
flat_curve 低
multimodal 低
best_at_search_edge 低
```

## 7. Spin Scan Observe-Only

`spin_scan_localization` 只用于原地旋转观测诊断。

它不控制电机，不输出 `cmd_vel`，不写回 pose。

使用条件：

```text
spin_scan_localization.enabled = true
spin_scan_localization.mode = observe_only
```

工作方式：

```text
检测机器人原地旋转
累积 front/left/right 三路 ToF
按 global bearing 分桶
形成稀疏 360 度 virtual scan
用 current grid 做只读 yaw matching
输出曲线和 summary
```

输出：

```text
spin_scan.csv
spin_scan_bins.csv
spin_match_curve.csv
spin_match_summary.csv
```

该功能用于判断：

```text
原地扫描是否能增强 yaw 可观测性
当前地图是否支持 ToF yaw matching
场景是否 flat / multimodal / edge-best
```

## 8. 日志输出要求

标准 run 目录必须包含：

```text
config.resolved.yaml
map.pgm
map.yaml
trajectory.csv
localization_log.csv
tof_log.csv
metrics.csv
```

开启 `tof_pose_correction` 时额外包含：

```text
tof_correction_log.csv
candidate_yaw.csv
yaw_residual_curve.csv
yaw_residual_summary.csv
```

开启 `spin_scan_localization` 时额外包含：

```text
spin_scan.csv
spin_scan_bins.csv
spin_match_curve.csv
spin_match_summary.csv
```

`tof_log.csv` 需要能看出：

```text
每路 ToF 原始 range/status/confidence
过滤后是否参与建图
被拒绝原因
read fail / gap / missing
mapping skipped reason
```

`metrics.csv` 需要能看出：

```text
tof_samples
map_updates
low_odom_quality_pauses
tof_unhealthy_pauses
front/left/right health
spin_scan_attempts
spin_scan_completed
spin_scan_matched
spin_scan_usable
```

## 9. 板端 Smoke 验收

板端 smoke 使用：

```text
/userdata/robot_slamd/robot_slamd
/userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml
```

运行命令：

```sh
/userdata/robot_slamd/robot_slamd \
  --config /userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml \
  --duration-s 20 \
  --output-dir /userdata/robot_slamd_board_sim_spin_scan_smoke
```

合格标准：

```text
程序运行 20s 不崩
latest 目录生成
config.resolved.yaml 存在
map.pgm / map.yaml 存在
trajectory.csv 存在
localization_log.csv 存在
tof_log.csv 存在
metrics.csv 存在
spin_scan.csv 存在
spin_scan_bins.csv 存在
spin_match_curve.csv 存在
spin_match_summary.csv 存在
```

## 10. 性能画像要求

如果板端有 `python3` 和 `file`：

```sh
python3 tools/run_board_runtime_smoke.py \
  --binary /userdata/robot_slamd/robot_slamd \
  --config /userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml \
  --output-dir /userdata/robot_slamd_board_sim_spin_scan_smoke \
  --duration-s 20 \
  --expected-md5 <md5>

python3 tools/summarize_runtime_profile.py \
  /userdata/robot_slamd_board_sim_spin_scan_smoke/latest \
  --strict
```

如果板端没有 `python3` 或 `file`，使用 shell fallback：

```sh
sh /userdata/robot_slamd/tools/board_runtime_profile_smoke.sh \
  /userdata/robot_slamd/robot_slamd \
  /userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml \
  /userdata/robot_slamd_board_runtime_profile_smoke \
  20

sh /userdata/robot_slamd/tools/board_summarize_runtime_profile.sh \
  /userdata/robot_slamd_board_runtime_profile_smoke/latest
```

性能 smoke 合格标准：

```text
duration_s > 0
rss_mb_max < 200
cpu_percent_max < 200
runtime_profile.csv 存在
```

2026-06-29 板端串口 smoke 实测：

```text
duration_s,20.000
cpu_percent_mean,4.590
cpu_percent_max,7.805
rss_mb_mean,2.309
rss_mb_max,2.309
thread_count_max,1
strict,pass
```

## 11. 下午模型导出交付物

前期模型导出建议包含：

```text
robot_slamd ARM binary
config/config.board_sim_spin_scan_smoke.yaml
config/config.hw_v1_5_smoke.yaml
config/config.hw_yaw_candidate.yaml
docs/BOARD_DEPLOY_SERIAL.md
docs/TOF_REQUIREMENTS.md
一次板端 smoke run_dir
map.pgm
map.yaml
trajectory.csv
localization_log.csv
tof_log.csv
metrics.csv
runtime_profile.csv
```

交付时必须同步：

```text
binary md5
运行配置 config.resolved.yaml
板端运行命令
板端 smoke 结果
性能摘要
当前限制：ToF 不写回 pose，只做建图和 observe-only 诊断
```

## 12. 当前限制

当前版本明确不做：

```text
不实现 yaw_only 写回
不实现 xy_yaw 写回
不实现 spin_scan 写回
不调用 apply_pose_correction
不控制电机
不输出 cmd_vel
不把 ToF 当成激光雷达
不承诺回环定位
```

当前版本重点是：

```text
真实硬件可采集
异常可观测
建图不被坏数据污染
结果可复现
板端可运行
性能可量化
```
