# C20S/C40D/C200T 云台台架判别探针

换机型/换固件后复验协议符号约定用。方法论文档见 `doc/gimbal_protocol_verified.md` 附录 C。
依赖：`/usr/bin/python3 -m pip install pyserial`（系统 python3）。

| 脚本 | 用途 |
|---|---|
| `probe_angle_domain.py` | LOCK+角度 ±10° 阶跃：姿态域符号、姿态↔编码器关系 |
| `probe_rate_fpv_domain.py` | 真实角速度 ±30°/s + FPV+角度 ±20°：速率命令域、FPV 相对角域 |
| `probe_direction_ab.py` | A/B 两段 ±30° 摆动：肉眼判别 cam+/op+ 的绝对抬头/低头方向 |
| `probe_manual_log.py <out.tsv>` | 60s 全遥测 50Hz 记录器：手动滚转/偏航激励后离线判符号 |
| `probe_go_zero.py` | go_zero 回中位行为（实测 C20S fw38: 完全无效，不回中不干扰） |
| `probe_fpv_to_lock.py` | FPV+角度0(fl_sens=15)→LOCK+速率 转换：回中原语与模式切换连续性 |
| `probe_aim_after_home.py` | 复刻节点 home_on_connect+AIM 速率律：回中后 AIM 收敛性 |

用法示例：
```
/usr/bin/python3 -I probe_angle_domain.py          # 直接跑, 云台在 /dev/ttyUSB0
/usr/bin/python3 -I probe_manual_log.py /tmp/log.tsv
```
注意：若上一工具退出时发过 STOP（如 gimbal_selftest/驱动），需先发 START（probe 脚本会在
首包发 cmd=2 的场合自行处理；手动场景用 probe_rate_fpv_domain.py 开头补发）。

## `mission_live.py` — MISSION 模式真机验证

真云台 + 真节点 + 假飞控(TCP) + 闭环 LOS 仿真（订阅 /fpv/gimbal 按固定世界目标计算 LOS）。
场景覆盖：AIM→MISSION 进入、目标收敛（−10°/+20°）、LOS 超时保持、退 MISSION→HORIZON 回平、
解除武装→AIM。用法：`ROS_DOMAIN_ID=<id> python3 mission_live.py`（云台需在 /dev/ttyUSB1，可改脚本）。
注意：armed 必须先于 mission 置位（同时置位会因 mission_blocked 锁存而进不了 MISSION——
真机流程中由拦截器保证此顺序）；假飞控 0x12 的 mode_flags 需带 OFFBOARD 位 (1<<13) 使 rc_manual=false。
