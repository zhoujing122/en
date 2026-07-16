

## 1. 结论先行

请底盘电机控制模块至少提供以下 6 项能力：

1. 初始化与启停：`init/start/stop/shutdown`，并能返回是否就绪。
2. 运动命令：接收方向、归一化速度、时间戳、TTL、来源、原因和序号，并同步返回“已接收 / 已拒绝 / 发送失败”。
3. 安全停车：提供幂等 `STOP` 和最高优先级 `EMERGENCY_STOP`；TTL 到期后必须由底盘模块自己自动停车。
4. 双轮反馈：提供左右轮累计编码器 ticks、实时有符号 RPM、电流、电机状态、采样时间和有效标志。
5. 底盘里程速度：提供由双轮反馈计算出的 `linear_velocity_mps` 和 `angular_velocity_rad_s`，同时保留左右轮原始反馈便于诊断。
6. 健康与故障：提供通信、超时、校验、状态、过流、堵转、反馈陈旧等状态及稳定错误码。


## 2. 当前项目边界

`robot_slamd` 当前已有两层相关接口：

- 上层运动接口：`SoftwareMotionCommandTransport::send_command()`，输入方向 + 归一化速度 + TTL，不要求算法模块直接编码 BL4820 寄存器或 UART 帧。
- 编码器读取实现：`CjcBl4820UartEncoder`，直接打开左右两个 UART，读取单圈位置、RPM、电流和状态，并在本地累计 ticks。

如果你负责的模块只接收运动命令、不占用电机 UART，则现有编码器读取逻辑可以保留。

如果你负责的模块会打开 BL4820 UART（这是推荐架构），则你必须同时输出编码器和电机状态反馈，`robot_slamd` 后续改为通过适配器读取你的反馈，不能再直接打开相同 UART。

```text
robot_slamd
  ├─ 运动命令 ───────────────> 底盘电机控制模块
  ├─ STOP / EMERGENCY_STOP ─> 底盘电机控制模块
  └─ 编码器/轮速/健康反馈 <── 底盘电机控制模块
                                  ├─ 独占左轮 UART
                                  └─ 独占右轮 UART
```

算法层不要求你实现 SLAM、建图、ToF 避障或定位器状态机。但底盘模块必须实现 TTL、急停、通信故障停车等本地安全机制，因为上层进程崩溃时不能依赖上层继续发送 STOP。

## 3. 必须暴露的运动命令接口

### 3.1 命令结构

接口语义应等价于当前代码中的 `SoftwareMotionCommand`：

```cpp
enum class MotionDirection {
    STOP = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    FORWARD = 3,
    BACKWARD = 4,
    EMERGENCY_STOP = 5,
};

enum class MotionSource {
    UNKNOWN = 0,
    ACTIVE_SCAN = 1,
    RECOVERY = 2,
    MANUAL_TEST = 3,
    SAFETY_STOP = 4,
};

struct MotionCommand {
    MotionDirection direction;
    double speed_normalized;  // [0.0, 1.0]
    double timestamp_s;       // 单调时钟，秒
    double ttl_s;             // 命令有效期，秒，必须 > 0
    MotionSource source;
    std::string reason;
    uint64_t sequence;
};
```

当前阶段实际只允许：

- `STOP`
- `TURN_LEFT`
- `TURN_RIGHT`
- `EMERGENCY_STOP`

`FORWARD` 和 `BACKWARD` 为后续预留，当前默认必须拒绝，不能因为枚举存在就直接驱动车体平移。

### 3.2 返回值

返回语义应等价于当前 `SoftwareMotionSendResult`：

```cpp
struct MotionCommandAck {
    bool ok;                    // 接口调用/传输是否成功
    bool accepted;              // 底盘是否接受该命令执行
    std::string error;          // 稳定、可记录的错误码
    uint64_t transport_sequence;
    double timestamp_s;         // 底盘接收/处理时刻，单调时钟秒
};
```

返回值必须按以下方式解释：

| `ok` | `accepted` | 含义 | 上层处理 |
| --- | --- | --- | --- |
| true | true | 命令通过校验并被底盘接受 | 继续监控反馈；不代表电机已经达到目标速度 |
| true | false | 接口正常，但命令被安全策略拒绝 | 视为未获得运动许可，随后发送 STOP |
| false | false | 调用、IPC 或底层发送失败 | 视为故障，随后发送 STOP / 进入故障态 |
| false | true | 非法组合 | 禁止返回 |

建议接口：

```cpp
MotionCommandAck send_command(const MotionCommand &command);
```

如果使用 C ABI、Unix socket、共享内存或其他 IPC，字段和语义必须保持一致，不要求必须使用上述 C++ 声明。

## 4. 命令执行语义

### 4.1 速度

`speed_normalized` 范围为 `[0.0, 1.0]`。底盘模块负责将其映射为实际轮速、档位或闭环速度目标，并提供可配置映射表。

当前项目的保守限制为：

| 用途 | 最大归一化速度 | 最大持续时间 |
| --- | ---: | ---: |
| 悬空方向探测 | 0.03 | 0.30 s |
| 悬空主动扫描测试 | 0.05 | 0.50 s |
| 当前软件接口硬上限默认值 | 0.10 | 由上层和 TTL 共同限制 |

底盘不得自行把很小的归一化速度映射成危险的高 RPM。映射关系和最小可稳定运行 RPM 需要在联调记录中固化。

### 4.2 STOP

- `STOP` 的 `speed_normalized` 必须为 `0.0`。
- 重复调用必须安全且返回成功；即 STOP 必须幂等。
- STOP 应清除当前普通运动目标和 TTL 计时器。
- 即使当前没有运动，也必须允许重复 STOP。
- 建议区分“受控减速停车”和“驱动失能”，但默认 STOP 的物理行为必须由双方确认。

### 4.3 EMERGENCY_STOP

- `EMERGENCY_STOP` 的 `speed_normalized` 必须为 `0.0`。
- 优先级高于全部普通命令，必须立即中断当前命令。
- 急停锁存后，普通运动命令必须被拒绝，直到显式复位。
- 建议另设 `clear_emergency_stop()`，且复位不能自动恢复急停前的运动命令。

### 4.4 TTL / 看门狗

- `ttl_s` 不是建议值，而是强制停车期限。
- 底盘模块接收非 STOP 命令后必须本地启动看门狗。
- 在 TTL 内没有收到更新命令时，底盘模块必须自动执行 STOP。
- 上层进程退出、IPC 断开、线程卡死、序号停止更新时，仍必须能自动停车。
- 当前默认 TTL 为 `0.30 s`。
- 不允许用“上一次速度一直保持”作为掉线后的默认行为。

若双方在同一 Linux 主机运行，时间戳统一使用 `CLOCK_MONOTONIC`。若不共享同一单调时钟域，TTL 必须以底盘模块的本地接收时刻启动，同时将跨设备时钟同步方式写入联调记录。

### 4.5 序号与重复命令

- `sequence` 按命令源单调递增。
- 相同序号的重传应幂等处理并返回同一结果，不能重复触发一次性动作。
- 小于最近已接受序号的普通运动命令应拒绝，错误码为 `stale_sequence`。
- STOP 和 EMERGENCY_STOP 不应因为序号陈旧而被忽略；安全停车优先。

## 5. 方向与坐标约定

机器人坐标系约定：

- `+X`：车头正前方。
- `+Y`：车体左侧。
- `+Z`：车体上方。
- 正角速度：从上往下看逆时针，即左转。
- 左/右轮正 RPM：该轮推动机器人向前。

当前代码约定：

| 动作 | 左轮目标 | 右轮目标 |
| --- | ---: | ---: |
| STOP | 0 | 0 |
| TURN_LEFT | 负 RPM | 正 RPM |
| TURN_RIGHT | 正 RPM | 负 RPM |
| FORWARD（暂不启用） | 正 RPM | 正 RPM |
| BACKWARD（暂不启用） | 负 RPM | 负 RPM |

注意：电机安装镜像、线序或厂家正方向可能使“电机正转”不等于“车轮向前”。底盘模块必须允许独立配置 `left_sign`、`right_sign`，对外反馈和命令统一转换成上述车体语义。

首次真机验证必须悬空进行：低速左转、STOP、低速右转、STOP，并同时核对命令方向、两轮 RPM 符号和编码器 ticks 增减方向。

## 6. 必须暴露的反馈接口

### 6.1 推荐反馈结构

```cpp
struct WheelFeedback {
    bool valid;
    int64_t total_ticks;            // 已按车体方向修正的累计有符号 ticks
    uint16_t position_raw;          // BL4820 单圈原始位置，诊断用
    double speed_rpm;               // 已按车体方向修正的有符号 RPM
    double current_a;               // A
    uint16_t current_raw;           // 原始值，诊断用
    uint32_t motor_status;          // 原始状态/故障位
    double sample_timestamp_s;      // 该轮估计采样时刻，单调时钟秒
    double request_start_s;
    double response_received_s;
    double read_latency_s;
    uint64_t sequence;
    std::string error;
};

struct ChassisFeedback {
    bool valid;
    WheelFeedback left;
    WheelFeedback right;
    double pair_timestamp_s;        // 左右轮采样时刻的中点
    double pair_skew_s;             // abs(left_time - right_time)
    double linear_velocity_mps;
    double angular_velocity_rad_s;
    bool command_active;
    bool command_settled;
    MotionDirection active_direction;
    double active_speed_normalized;
    uint64_t active_command_sequence;
    bool fault;
    std::string fault_reason;
};
```

建议接口：

```cpp
bool read_feedback(ChassisFeedback &out);
```

也可以采用 100 Hz 发布/订阅方式。无论拉取还是推送，都必须保证每帧带时间戳、序号和有效标志，不能只返回一组无时间信息的 RPM。

### 6.2 编码器 ticks

优先对外提供累计有符号 `int64 total_ticks`。当前电机只给单圈绝对位置时，底盘模块按以下规则展开：

```text
delta = current_position - last_position
if delta > counts_per_rev / 2:  delta -= counts_per_rev
if delta < -counts_per_rev / 2: delta += counts_per_rev
total_ticks += wheel_sign * delta
```

当前配置按 `counts_per_rev = 1024` 处理。第一帧只建立基准，累计 ticks 从 0 开始，不应把第一帧的绝对位置当成已行驶距离。

必须拒绝等于半圈或无法判断方向的跳变，不能把异常帧提交到累计值。进程重启、设备重连或电机重新上电后，必须显式标记 `encoder_reset`，上层不能把重置前后的累计值直接相减。

如果暂时只能提供 `position_raw`，还必须提供每轮独立采样时间、有效标志和重置标志，由 `robot_slamd` 继续做 unwrap；但这不是首选交付形式。

### 6.3 底盘线速度与角速度

对外单位固定为 SI：

```text
v_left  = left_rpm  / 60 * 2π * left_wheel_radius_m
v_right = right_rpm / 60 * 2π * right_wheel_radius_m
linear_velocity_mps  = (v_left + v_right) / 2
angular_velocity_rad_s = (v_right - v_left) / wheel_base_m
```

`linear_velocity_mps` 向前为正；`angular_velocity_rad_s` 左转为正。

当前 SLAM 后端的 `WheelOdomFrame` 需要：

```cpp
struct WheelOdomFrame {
    double timestamp_s;
    double linear_mps;
    double angular_rad_s;
    bool valid;
};
```

仅输出该聚合帧可以驱动现有里程推算，但不能满足电流、堵转、左右轮通信和方向诊断，因此仍需保留逐轮反馈。

### 6.4 反馈频率与时延

| 项目 | 当前项目期望 |
| --- | ---: |
| 编码器读取 / 反馈频率 | 100 Hz |
| 单轮 UART 超时 | 10 ms |
| 单轮最大读取延迟 | 10 ms |
| 左右轮最大采样偏差 | 10 ms |
| 上层允许的编码器最大年龄 | 0.5 s（安全兜底，不代表正常性能目标） |

左右轮当前为顺序查询，因此必须分别记录请求开始与响应完成时刻；每轮估计采样时刻可使用请求窗口中点，双轮帧时间可使用两轮估计采样时刻中点。

一侧读取失败时不得把一新一旧两轮数据拼成“有效双轮反馈”。应保留上次累计值用于诊断，但本周期 `ChassisFeedback.valid=false`。

## 7. BL4820 当前读取协议

当前项目使用左右轮独立 UART：

| 项目 | 值 |
| --- | --- |
| 波特率 | 1,000,000 bps |
| 数据格式 | 8N1，无校验，无流控 |
| 电机 ID | 左右轮均为 `0x01`，因串口物理隔离 |
| 读起始地址 | `0x24` |
| 读长度 | `0x06` |
| 数据 | position、speed、current，各 2 字节，小端 |

请求帧：

```text
FF FF 01 04 02 24 06 CE
```

期望返回：

```text
FF FF ID 0B STATUS 04 02 24 POS_L POS_H RPM_L RPM_H CUR_L CUR_H CHECKSUM
```

解析约定：

- `position_raw`：`uint16` little-endian。
- `speed_rpm`：`int16` little-endian，二补码。
- `current_raw`：`uint16` little-endian，当前代码按 `1 = 0.1 A` 换算。
- `STATUS == 0` 才接收该帧。
- 校验：`~(ID + Length + 后续字段...)`，取低 8 位。

更完整的读协议确认项见仓库根目录 `通讯协议_项目化版本.md`。电机写控制协议、UART 电平、状态位定义、电流比例和最大响应时延仍需底盘负责人依据最终硬件资料确认，不能仅凭当前示例假定。

## 8. 健康状态与错误码

建议接口：

```cpp
struct ChassisHealth {
    bool ready;
    bool live_enabled;
    bool emergency_stop_latched;
    bool left_healthy;
    bool right_healthy;
    uint64_t left_read_errors;
    uint64_t right_read_errors;
    uint64_t checksum_errors;
    uint64_t timeout_errors;
    uint64_t status_errors;
    uint64_t frame_errors;
    uint64_t pair_skew_errors;
    std::string state;
    std::string last_error;
};

ChassisHealth get_health();
```

错误码必须稳定、机器可判定，建议至少包含：

- `not_initialized`
- `not_ready`
- `live_motion_disabled`
- `invalid_direction`
- `invalid_speed`
- `invalid_ttl`
- `stale_timestamp`
- `stale_sequence`
- `translation_disabled`
- `emergency_stop_latched`
- `left_uart_timeout` / `right_uart_timeout`
- `left_checksum_error` / `right_checksum_error`
- `left_status_error` / `right_status_error`
- `encoder_pair_skew_exceeded`
- `encoder_jump_rejected`
- `encoder_reset`
- `feedback_stale`
- `motor_overcurrent`
- `motor_stall`
- `motor_overtemperature`
- `motor_voltage_fault`
- `transport_error`

错误字符串用于日志，若协议允许，最好同时提供稳定的整型错误枚举。不要把厂家原始状态位丢掉；应同时输出归一化错误码和 `motor_status` 原值。

## 9. 本地安全要求

底盘模块必须本地实现：

1. 默认上电不运动，`live_enabled=false`。
2. 初始化未完成、反馈无效或串口未就绪时拒绝非 STOP 命令。
3. TTL 到期自动 STOP。
4. IPC 断开、调用方退出或心跳丢失自动 STOP。
5. EMERGENCY_STOP 最高优先级并锁存。
6. 左右轮任一通信连续失败达到阈值时停止两轮，而不是只让单轮继续转。
7. 电机状态异常、过流、堵转或反馈陈旧时停止并上报故障。
8. 所有非有限数值（NaN/Inf）、越界速度和无效 TTL 必须拒绝。
9. STOP 失败必须作为最高级故障上报，不能返回假成功。
10. 提供 shadow/dry-run 模式：完整校验和记录命令，但不写真实电机。

当前项目侧还会执行 ToF 障碍物、定位状态、编码器新鲜度、电流、堵转、命令陈旧和 deadman 等检查；这些上层检查不能替代底盘本地安全。

## 10. 生命周期与并发要求

建议提供：

```cpp
bool init(const ChassisConfig &config, std::string &error);
bool start(std::string &error);
bool ready() const;
MotionCommandAck send_command(const MotionCommand &command);
bool read_feedback(ChassisFeedback &feedback);
ChassisHealth get_health() const;
MotionCommandAck emergency_stop(const std::string &reason);
bool clear_emergency_stop(std::string &error);
void shutdown();
```

要求：

- `init/start/shutdown` 可重复调用时行为明确，不泄漏串口句柄或后台线程。
- `shutdown()` 必须先 STOP，再关闭串口。
- `send_command()` 与 `read_feedback()` 可以由不同线程调用；实现必须线程安全，或在接口文档中明确要求单线程串行调用。
- 反馈读取不能长时间阻塞命令发送，EMERGENCY_STOP 不能排在普通查询后等待很久。
- UART 读写需要统一调度，避免写控制帧和读状态帧互相串帧。
- 对外 ABI/API 需要版本号和 capability 查询，便于以后增加平移、制动方式或新状态字段。

## 11. 静态参数与能力查询

底盘模块应能返回或记录以下参数：

- 左右轮设备路径、波特率和电机 ID。
- `counts_per_rev`。
- 左右轮方向修正 `left_sign/right_sign`。
- 左右轮半径。
- 轮距 `wheel_base_m`。
- 归一化速度到目标 RPM 的映射。
- 最大 / 最小允许 RPM。
- 控制频率、反馈频率、UART 超时和连续错误阈值。
- STOP 的物理方式：速度置零、受控减速、锁电机或失能。
- 是否支持平移、原地旋转、急停锁存、硬件时间戳、累计 ticks。

需要特别注意：当前代码中通用底盘参数 `wheel_base_m` 默认是 `0.180 m`，运动执行参数 `motion_execution_wheel_base_m` 默认是 `0.160 m`。这两者必须在真机测量后统一，否则命令角速度和里程计角速度会使用不同几何模型。当前轮半径默认是 `0.032 m`，也必须实测标定。

## 12. 与现有 C++ 代码的最小适配

### 12.1 命令侧

最直接的接入方式是实现：

```cpp
class ProductionSoftwareMotionTransport
    : public robot_slamd::SoftwareMotionCommandTransport {
public:
    robot_slamd::SoftwareMotionSendResult send_command(
        const robot_slamd::SoftwareMotionCommand &command) override;
};
```

输入字段和返回字段已在第 3 节列出。这个适配器内部可以调用你的动态库、Unix socket 或本地服务。

### 12.2 反馈侧

反馈至少需要能转换成两类现有数据：

```cpp
robot_slamd::EncoderSample {
    t_us,
    left_ticks,
    right_ticks
}
```

以及：

```cpp
robot_slamd::WheelOdomFrame {
    timestamp_s,
    linear_mps,
    angular_rad_s,
    valid
}
```

逐轮 RPM、电流、状态、读取延迟和左右轮时间偏差还会进入 `EncoderStats` 与运动安全判断，因此接口中不能省略这些诊断字段。

## 13. 联调验收用例

交付前至少通过以下测试：

### 13.1 纯软件 / shadow

- 非法方向、NaN/Inf、负速度、速度大于 1、TTL 小于等于 0 均被拒绝。
- 当前阶段 FORWARD/BACKWARD 被拒绝。
- STOP 可重复调用且始终安全。
- EMERGENCY_STOP 能抢占普通命令，锁存后普通命令被拒绝。
- 普通命令超过 TTL 后，无需上层再发消息即可自动 STOP。
- 旧时间戳、旧序号和重复序号符合第 4.5 节语义。
- `ok/accepted/error` 三者组合符合第 3.2 节。

### 13.2 编码器与协议

- 请求 `FF FF 01 04 02 24 06 CE` 能稳定返回。
- `9C FF` 解析为 `-100 RPM`。
- `1000 -> 20` 展开为 `+44 ticks`。
- `20 -> 1000` 展开为 `-44 ticks`。
- 校验错、状态非 0、超时、半圈歧义跳变均不更新累计 ticks。
- 单轮失败时整帧 `valid=false`，不混用新旧轮数据。
- 进程重启 / 电机重连能上报 `encoder_reset`。
- 连续运行时反馈频率、单轮延迟和左右轮采样偏差达到第 6.4 节要求。

### 13.3 悬空真机

- 上电默认不转，先验证 STOP。
- `TURN_LEFT`：左轮负、右轮正，编码器方向与 RPM 符号一致。
- `TURN_RIGHT`：左轮正、右轮负，编码器方向与 RPM 符号一致。
- 测试速度不超过 0.03，单次不超过 0.30 s。
- 命令过程中断上层进程，底盘能在 TTL 内自动停车。
- 拔掉任一电机通信，两个轮都进入安全停止并正确上报故障。
- 急停后不会因旧命令、重连或清除急停而自行恢复运动。

所有悬空测试通过、方向和 STOP 行为书面确认之前，不允许落地运动。

## 14. 交付物清单

请底盘负责人最终提供：

1. 接口头文件或 IPC 协议定义，以及接口版本号。
2. 可链接库 / 服务程序和目标板构建方式。
3. 初始化、发送 TURN、发送 STOP、读取反馈的最小示例。
4. 所有错误码、厂家状态位和故障恢复方式说明。
5. 归一化速度到实际 RPM 的映射及限幅参数。
6. TTL、掉线停车、急停锁存和复位的实现说明。
7. UART 所有权、左右轮设备路径、电平、波特率、ID 和协议版本。
8. 编码器计数、方向修正、轮径和轮距标定值。
9. shadow 测试、协议测试和悬空真机测试记录。

## 15. 双方联调前必须确认的问题

以下问题当前代码或现有资料尚未完全冻结：

1. 最终 UART 电平是 3.3V TTL 还是 5V TTL。
2. BL4820 写速度 / 停止 / 失能的最终帧格式与校验方式。
3. `motor_status` 每个 bit 的厂家定义。
4. `current_raw` 是否确定为 `1 = 0.1 A`。
5. 单圈位置合法范围是 `0..1023`，还是可能返回 `1024`。
6. STOP 是速度置零、刹车锁定还是驱动失能，以及各种故障下采用哪一种。
7. 实测轮距应采用 0.160 m、0.180 m 还是新标定值。
8. 左右轮有效半径与负载下的标定值。
9. `speed_normalized` 到左右轮目标 RPM 的最终映射。
10. 底盘模块采用同进程动态库还是独立进程 IPC；若为 IPC，双方是否共享 `CLOCK_MONOTONIC` 时钟域。

以上项目确认后，应把结果写入配置和联调记录，不要只保留在口头约定中。
