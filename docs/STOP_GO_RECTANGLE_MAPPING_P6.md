# Stop-Go Rectangular Mapping P6

P6 基于 `6b357b0cf02d2423ff455ec73a17a39fb8397ed4`（P5：`Add stop-go single-corner transition and wall reacquisition`）完成。工作树为
`/home/lab/Desktop/小恩/worktrees/en_stop_go_rectangle_p6`，分支为
`feature/stop-go-rectangle-mapping-p6`。

## 系统边界

实际调用链仍然只有一条：

`Simulation / Replay Adapter -> RobotSlamSensorSnapshot -> RobotSlamApplication -> SparseSlamRuntimeCore -> StopGoMappingController -> RelativeMotionStepPort`

P5 的一个 `RobotSlamApplication`、一个 `SparseSlamRuntimeCore`、一个
`WheelImuDeadReckoning2D`、一个 `TimedOdomPoseBuffer`、一个 `map_T_odom` owner、一个
Sparse Map 和一个 Controller 均保留。Core 的 gyro bias、wheel baseline、odom 初始化、
map_T_odom 初始化和 Core Ready 门控没有搬到 Controller。Core 未 Ready 时仍只输入静止
Wheel/IMU，`commands_submitted=0`，不会捕获起点或判断墙角。

`single_corner` 继续使用 P5 行为并在一次转换后以 `single_corner_complete` 结束；
`rectangle_loop` 复用同一套 P5 状态机，循环完成四次转换，不复制 Controller 或 SLAM。

## 墙段循环和安全门

运行从墙段 1 开始。每次通过完整的候选、停止确认、右侧空间检查、固定 RIGHT 90°、
实际 odom yaw 验证、有限残余补偿、转后传感器验证和新左墙拟合后，才提交一次转换：

`1 -> 2 -> 3 -> 4 -> 5`

`wall_segment_id` 和 `corner_transition_id` 独立维护；普通 map revision 增加不会清空
墙模型，`frame_transform_epoch` 变化仍按 P5 规则使活动模型失效并回到安全恢复路径。
每个新墙段只有在墙模型有效、达到最小前进步数和实际 odom 距离、且前方离开危险区后才
重新 arm 墙角检测，因此同一墙角不会被重复计数。

墙角只允许固定 `RIGHT`，即 action 1、target 90°。右侧 ToF 只是安全门，不参与决定方向。
触发距离使用：

`minimum_trigger = max(0, sweep_radius + clearance_margin - front_tof_offset) + forward_step + expected_overrun`

配置解析和启动校验拒绝低于该值的触发阈值；Invalid/Unspecified 右侧读数不会被当作空旷，
右侧空间不足、转后新墙丢失、转角验证失败均停止并输出明确失败原因。

主转向完成判据只使用 Core 连续 Wheel/IMU 估计的 `odom_T_base` yaw。欠转使用有界 RIGHT
补偿，过转使用有界 LEFT 补偿；每次补偿都要经过新的 command id、Done、MappingSettleGate
和再次 yaw 验证。requested 90°和命令结果目标量都不作为实际转角或 odometry。

## 起点、实际距离和闭环

起点锚点只在 Core Ready、初始停稳采样和第一墙模型有效、第一条主转向尚未发出时捕获，
包括估计 map/odom pose、frame epoch、初始墙 heading、base-to-wall distance、fit 质量和
签名。普通命令和墙角转换不重设锚点。

矩形运行距离由连续 `odom_T_base` 位姿增量累计，拒绝异常跳变，不使用 step 数、命令距离、
RPM 或 Ground Truth 替代。四次转换只是闭环候选的必要条件；还必须满足累计距离、第四段
恢复步数/距离、当前估计 map pose 的位置和 yaw 误差，以及 segment 5 与 segment 1 的墙
方向、距离和线偏移一致性。

候选成立后进入 `RECTANGLE_CLOSURE_CONFIRMING`：停止、等待 MappingSettleGate、原地多次
稳定三 ToF 检查，确认阶段 `mapping_write_enabled=false` 且 map revision 不增加。确认失败
受最大尝试次数限制；第四墙角后的搜索也受 step、odom distance、总步数和 runtime 上限限制。
没有 pose snap、rectangle geometry snap、墙点替换、map_T_odom 写回或理想矩形吸附。

确认成功后进入最终停稳，允许一次最终正式稳定 ToF 地图提交，检查 Core idle 后调用现有
`SparseSlamRuntimeCore::save_sparse_map`，随后用现有 artifact loader 重载并校验 map id、
run id、revision、grid resolution、ToF 外参、轮参数、cell 数和 checksum。完成后不再提交运动。

## 地图质量与 Ground Truth 隔离

`RectangleMapQualityReport` 是只读验收器，不是算法输入。它在运行结束后才读取仿真 Ground
Truth 和最终 Sparse Map，测量可观测墙覆盖率、占据/自由/未知 cell、p95 和最大墙厚、ghost
occupied 比例、重复墙带数量、map checksum 及真实最终位姿误差。Controller/Core 均不读取
Ground Truth；Ground Truth 只用于传感器生成、碰撞检查和结束后的 evaluator。

名义场景一次确定性结果：

- transitions=4，segment sequence=`1,2,3,4,5`，主 RIGHT=4；
- Formal helper nominal estimated closure position error `0.0367239 m`，yaw error
  `-0.168197°`；正式 YAML CLI nominal 为 `0.0480898 m` 和 `0.954601°`；
- Formal helper observable wall coverage `0.518248`；正式 YAML CLI 为 `0.492701`。
  两者 p95 wall thickness 均为 `1` cell，ghost ratio=`0`，
  duplicate wall bands=`0`，collision=`0`；
- corner confirm/turn/turn-verify/closure-confirm map writes 均为 `0`；
- map save/reload 成功，revision 和 checksum 一致。

以上数值和尺寸均为 Simulation provisional 验收，不是实机标定结果。

## Simulation、Replay 和对比

矩形 Simulation 使用确定性局部矩形房间。机器人只把模拟 Wheel、IMU、三 ToF 和 Core
estimated pose 提供给算法；房间尺寸、角点位置和理想路径不进入 Controller。

Formal matrix 覆盖 nominal rectangle、mixed residuals、middle-segment false front、right
clearance failure、new wall loss、closure not reached 和 premature start proximity；失败
场景验证停止、转换计数不跳增且不保存为成功地图。现有 Replay codec/runner 扩展了 segment、
transition、closure 和 final mapping commit 字段，不发送运动，重放相同 sensor/motion
records。名义 Simulation 两次和 Replay 两次的 command/segment/closure/checksum 结果一致。

同一矩形世界另行运行现有 Frontier/A* exploration，产生只读
`RectangleMappingComparisonReport`。对比不修改任何算法、不参与 Stop-Go 决策；当前 Frontier/A*
运行按其真实终止语义记录（本次配置下为 `exploration_sparse_core_step_failed:capacity_exceeded`），
不改写为 rectangle complete。

## 配置和限制

正式配置为 `config/stop_go_rectangle_sim.yaml`，复用现有解析和校验体系。P5 corner 参数、
扫掠半径、过冲、ToF offset、右侧安全距离、闭环阈值、重采样上限、保存路径和 map-quality
阈值全部来自配置；矩形 target/max transitions 强制为 4，确认 required passes 不得超过
attempts。Simulation YAML 的 minimum observable wall coverage 当前为 `0.45`；房间尺寸、
传感器量程、触发阈值、墙距、转向过冲、覆盖阈值和质量阈值都是
Simulation provisional。Real 不继承这些 provisional 参数，尺寸、过冲和输出边界未完成真实
标定时 fail-closed。

P6 只证明 Simulation/Replay 的矩形闭环、地图保存重载和确定性质量验收，不能证明真机已经
能够安全完成矩形房间。尚未证明真实最小可靠步长、真实 90°精度、转弯扫掠半径、前方阈值、
ToF 量程/盲区/无回波、三 ToF 精确外参或真机闭环误差。

下一阶段不再增加大型仿真功能，转向整合 P1.5 电机 Transport、真实三 ToF Adapter、真机
低速标定、单直墙、单墙角和最终矩形房间真机验收。
