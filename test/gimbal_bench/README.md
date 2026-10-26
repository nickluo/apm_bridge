# C20S/C40D/C200T 云台台架判别探针

换机型/换固件后复验协议符号约定用。方法论文档见 `doc/gimbal_protocol_verified.md` 附录 C。
依赖：`/usr/bin/python3 -m pip install pyserial`（系统 python3）。

| 脚本 | 用途 |
|---|---|
| `probe_angle_domain.py` | LOCK+角度 ±10° 阶跃：姿态域符号、姿态↔编码器关系 |
| `probe_rate_fpv_domain.py` | 真实角速度 ±30°/s + FPV+角度 ±20°：速率命令域、FPV 相对角域 |
| `probe_direction_ab.py` | A/B 两段 ±30° 摆动：肉眼判别 cam+/op+ 的绝对抬头/低头方向 |
| `probe_manual_log.py <out.tsv>` | 60s 全遥测 50Hz 记录器：手动滚转/偏航激励后离线判符号 |

用法示例：
```
/usr/bin/python3 -I probe_angle_domain.py          # 直接跑, 云台在 /dev/ttyUSB0
/usr/bin/python3 -I probe_manual_log.py /tmp/log.tsv
```
注意：若上一工具退出时发过 STOP（如 gimbal_selftest/驱动），需先发 START（probe 脚本会在
首包发 cmd=2 的场合自行处理；手动场景用 probe_rate_fpv_domain.py 开头补发）。
