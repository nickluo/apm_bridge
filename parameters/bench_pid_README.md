# 三轴测试架 PID 整定

`bench_pid_tuner` 是**显式启用的真机台架实验节点**，不是空中自动调参器。
它不解锁、不启动视觉任务、不发送 `SAVE`，也不替代物理急停。
禁止在自由飞行中运行。机架必须可靠固定在三轴架上，转轴附近无碰撞风险，
动力、固定件、桨叶防护和独立断电/ARM 开关均须由操作者确认。
拆桨可以验证通信/取消，但没有可测扭矩响应时，整定会失败，不能声称得到有效 PID。

## 依赖和构建

需要本次修改后的 Betaflight 固件、`quadrotor_msgs` 和 `apm_bridge`：

```bash
# 在 Betaflight 仓库按实际板卡重新构建/刷写；不要将 SITL 固件刷入真机。
# make CONFIG=<实际板卡>

source /opt/ros/jazzy/setup.bash
cd ~/workspace/fpv_ws
source install/setup.bash
colcon build --packages-select quadrotor_msgs apm_bridge --symlink-install
source install/setup.bash
```

新增 CLI 定制协议命令 `CMD_APPLY_PID=5`：

- `args="check"` 返回 `PID PROFILE <index>`，不修改系数。
- `args="<index>"` 校验当前 profile，调用 `pidInitConfig` 和 `pidResetIterm`，
  将当前 profile 的 RAM 参数应用到运行中的控制器，不写 Flash。
- 已解锁时，只在新鲜 OFFBOARD、零油门、零角速度指令且三轴实测角速度均小于
  5 deg/s 时允许应用。否则报错。未解锁时允许恢复原参数。
- 旧固件会拒绝命令；节点必须停止，绝不把普通 `SET` 的读回误当成即时生效。

这是对已有 GET/SET/INFO/SAVE 的向后兼容扩展；旧命令编号不变。节点启动前保存
`dump all`/`diff all`，使用独立 PID profile，并禁用该 profile 的 simplified PID
调整（`simplified_pids_mode=OFF`）。不要在实验中切换 profile 或使用其它 CLI 写入者。
节点不会改动 F、滤波、D Max、TPA、机体资源或解锁/GPS 保护；这些设置会影响实验结果。

## 连接与通道

先按实际串口启动 bridge，例如使用现有 `custom_link.launch.py`；不要启动
`drone_interceptor`、电机校准或其它控制发布者。节点要求：

- `/mavros/rc/in`、`/mavros/state`、`/mavros/imu/data`、`/fpv/failsafe` 新鲜；
- `/fpv/cli` 服务可用；不依赖 bridge 的节点名称或参数服务。
- `tuning_channel` 与 bridge 的 `channel_trigger` 一样使用**从0开始**的通道索引，
  范围 0..15。默认 `8` 对应真实遥控器 **CH9 / AUX5**；例如 `11` 对应 CH12。
  所选通道低于 1100 后，再拨到高于 1900 启动；启动时高电平不会自动开始，
  保持高不会重复触发。拨低取消，通道缺失会停止试验并要求重新经历低→高。
- 节点只读取所选整定通道，**不解释其它 RC 通道或摇杆的功能/阈值**；不再配置
  `arm_channel`、`mode_channel`、`offboard_channel`、`mission_channel`，也不读取
  bridge 的 `channel_trigger`。操作者须核对所选开关不与 ARM、飞行模式或 MISSION
  等功能共用，节点不自动检查这些映射。
- 飞控必须由飞手独立解锁且处于 ACRO；整定期间须维持 OFFBOARD 权限。
  节点通过飞控 state、failsafe 和 MISSION 状态检查这些条件；模式/摇杆接管由飞控
  仲裁，只有飞控报告权限丢失、状态异常或整定开关取消时才相应停止，
  不再通过其它 RC 值推断飞手意图。

节点只请求 OFFBOARD 和发布原始 body-rate/归一化油门，均由 bridge 经 custom link
传给飞控。**不发布解锁指令**。其它控制发布者在运行中出现，也会停止实验。

## 未装桨、未上架时的通路检查

此阶段保留 `enabled=false`、`rig_confirmed=false`，ARM 上锁、最低油门：

- 可以检查真实 RC/IMU/state/failsafe 遥测、profile 与 P/I/D 读取、同值 SET 回读，
  以及**未解锁条件下**的 APPLY_PID。不得发解锁、OFFBOARD、激励油门或 SAVE。
- 所选整定开关低→高只能用于观察禁用节点的拒绝日志，再拨低；不要为走通流程而绕过
  上架确认、未解锁保护或 GPS/ARM 条件。软件回调测试不能替代真实开关验证。
- 无桨且没有可测机械响应时，不能验证整定质量、完整激励或实际停桨时间。
- GET/INFO 和 `APPLY_PID check` 是无副作用查询，单次超时最多额外重试两次并记录警告；
  固件错误立即失败，取消/失去权限后不继续重试。SET 和实际 APPLY_PID **不重试**，
  超时仍按写入结果未知处理并要求上锁恢复。重试不代表写入稳定性已验收。
  当前 UART 接收解析器的帧内字节间隔超时最小为 2 ms；USB 转串口若产生分包停顿，
  有效长帧也可能被丢弃。发现偶发超时应先定位串口/分包/接线，再进行带动力试验。

## 参数与启动

参数文件为 `bench_pid.yaml`，默认 `enabled=false`、`rig_confirmed=false`，不驱动输出：

```bash
ros2 launch apm_bridge bench_pid.launch.py
```

复制并评审参数文件，确认测试架后，通过 launch 参数显式启用，无需修改 YAML：

```bash
ros2 launch apm_bridge bench_pid.launch.py \
  parameters:=/绝对路径/bench_pid.yaml enabled:=true rig_confirmed:=true
ros2 topic echo /bench_pid_tuner/status
```

`enabled`、`rig_confirmed` 两个 launch 参数均默认为 `false`，以布尔类型覆盖 YAML
中的同名值；即使 YAML 为 `true`，未显式传入启动参数仍保持禁用。支持分别设置
`enabled:=false` 或 `rig_confirmed:=false`，且只有两者都为 `true` 才允许整定。
其它参数仍从 `parameters` 指定的 YAML 加载。

`rig_confirmed` 只是人工确认项，**不是物理互锁传感器**。

| 参数 | 默认 | 含义 |
| --- | ---: | --- |
| `axes` | roll, pitch, yaw | 可先选单轴；按序测试，不同时激励多轴。 |
| `tuning_channel` | 8 | 整定启动/取消开关，从0开始的 RC 索引（0..15）；8=CH9。 |
| `throttle` | 0.12 | **归一化原始油门**，不是推力加速度；上限固定限制为 0.2。不是本机已验证值，需要已有动力/测试架评审。 |
| `rate_dps` | 5 | 小幅正负角速度激励，支持 1..10 deg/s。 |
| `pulse_s` / `tail_s` | 0.3 / 0.8 | 每方向激励和零率回落观察时间。 |
| `max_tilt_deg` | 15 | 相对实验初始姿态的任一欧拉轴偏移限值，含 yaw；并非主动回中控制。 |
| `max_rate_dps` | 40 | 任一轴实测角速度超限即取消。 |
| `max_gain_change` | 3 | P/I/D 每项始终在原值附近的变化上限，非每轮累加。 |
| `improvement` | 0.10 | 候选独立复测须比两次基线中的更优一次至少改善 10%。 |
| `timeout_s` | 180 | 会话总时间限制；失败退出，不自动延长实验。 |
| `output_dir` | ~/.ros/bench_pid | JSON 报告，以及正常完成时生成的同名 `.cli` 参数脚本。 |

## 算法和结果

参考仿真率响应测试的双向激励、跟踪误差、超调、回落和独立复测方法：

1. 查询当前 profile、保存全部三轴原始 P/I/D 到报告，之后才允许写参数。
2. 原参数测基线；每轴局部搜索 P、I、D，候选最多为原值上下各
   `min(max_gain_change, ceil(原值×10%))`。零 I/D 保持零，不凭空启用。
3. 候选应用前先零油门等待静止；SET 精确回读、再 GET 验证，之后执行 APPLY_PID。
4. 在小幅双向激励下评分；缺少响应、超调、回落振荡、交叉轴运动或遥测异常时
   **中止整个实验**，不继续探测更激进的候选。
5. 重测原参数，再独立重测候选；未获得可重复改善则保留原参数。
6. 成功结果留在 RAM，导出 CLI 文件，输出零油门/零率并退出 OFFBOARD；
   **不会自动上锁，也不保证停桨**：FC 仍解锁时可能维持怠速，飞手须立即独立上锁。
   本节点绝不自动向飞控发送 `SAVE`。

### 正常完成后的 CLI 文件

默认目录 `~/.ros/bench_pid/` 中，每次正常完成产生同一时间戳的两个文件：

- `bench-<时间戳>.json`：原值、最终值、试验数据和恢复状态；`cli_file` 记录脚本路径。
- `bench-<时间戳>.cli`：可粘贴到 Betaflight CLI 的 `profile <索引>` 和三轴 P/I/D
  `set` 命令，profile 索引从 **0** 开始。未整定轴保留该次会话原值；
  没有可重复改善时也导出最终保留的原值。

脚本不修改 F、滤波、D Max、TPA 或其它配置，要求导入目标保留实验时的这些设置，
且 `simplified_pids_mode=OFF`。它不是完整配置备份，也不是可直接自由飞行的认证参数。
脚本末尾的 `# save` **只是注释**，不会自动写 Flash 或重启。

操作顺序：使用独立 ARM 开关上锁，整定开关拨低，确认电机停止；评审 JSON 和原始完整备份，
核对目标固件、机体及 profile，再粘贴 CLI 脚本。逐项核对读回，确认采用后手工输入
`save` 保存并重启。不要在测试节点工作时导入，或把带桨已解锁状态当成可安全操作 CLI。
正常结束时状态日志会打印脚本绝对路径；报告中 `persisted=false` 表示**飞控 Flash 未保存**，
不表示主机文件未保存。

取消/失败不导出成功脚本；CLI 或最终报告写入失败时，本次会话按失败处理并保留恢复要求，
必须按下节说明上锁后恢复。只有脚本和报告都写入成功且最终状态检查通过，才接受结果。

这是有界局部搜索，不是系统辨识或保证全局最优的“自动调好”。尤其 I 的负载适应、
机架摩擦/惯量耦合和 D 的高频噪声，不能仅靠短脉冲或 200 Hz 遥测充分验证。
台架 PID 不能未经后续验证直接用于自由飞行；不自动调整 F 或滤波来掩盖失败。

## 取消、异常与恢复

整定开关拉低/所选通道缺失、RC/IMU/state/failsafe 超时、飞控报告 OFFBOARD 权限丢失、
FC 上锁、包线超限或 competing MISSION 都会停止试验：输出零油门/零率并撤销
主机 OFFBOARD 请求。其它 RC 通道不会直接触发节点取消。这个动作
**不保证已解锁电机物理停转**；怠速/接收机信号仍由 FC 仲裁，必须使用独立 ARM/断电急停。

有未确认的参数写入时，报告标记 `rollback_required=true`，禁止新一轮实验。
取消后将**整定开关保持低并独立上锁**，节点确认新鲜上锁状态后才恢复并 APPLY 原 PID。
不能在飞手已接管、仍解锁时抢回权限来恢复。链路/恢复失败会明确报错并锁止，不无限重试；
使用报告的原值手动恢复原 profile，或者在确认 Flash 中备份正确后重启。
断电或杀进程不能保证回滚；原始 PID 报告是必要的恢复依据。

没有机温/电流传感器互锁，油门上限与短时试验不能替代热保护。操作者全程在场。
测试通过只表示软件协议/状态机覆盖，未进行真实带桨测试架整定认证。
