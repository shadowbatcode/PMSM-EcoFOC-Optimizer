# Document and Component Result Comparison

This report compares the Word/legacy Chapter 3 metrics with the current R2025b component Simulink model outputs.

| 指标 | 文档传统PID/PI | 文档所提方法 | 当前组件传统PID/PI | 当前组件所提方法 |
|---|---:|---:|---:|---:|
| 速度超调量 | 2.97% | 2.94% | 11.09% | 9.01% |
| 调节时间 | 0.0461 s | 0.0451 s | 0.5906 s | 0.5569 s |
| 稳态转速误差 | 0.0000 r/min | 0.0149 r/min | -0.0000 r/min | -0.0514 r/min |
| 稳态输入功率 | 227.658 W | 227.397 W | 772.938 W | 764.765 W |
| 定子电流幅值 | 3.2577 A | 3.2236 A | 9.9244 A | 9.5741 A |
| 效率 | 94.406% | 94.517% | 84.71% | 85.62% |
| i_d^* 收敛时间 | 不适用 | 0.5503 s | 不适用 | 3.6015 s |
| 负载扰动恢复时间 | 0.0380 s | 0.0375 s | 0.0677 s | 0.0394 s |
