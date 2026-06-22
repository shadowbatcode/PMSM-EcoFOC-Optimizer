# 附录 A 核心程序代码

本附录截取 PMSM PID/PI 矢量控制与无模型反馈能效自寻优仿真的核心程序片段。完整工程由 MATLAB/Simulink R2025b 组件模型实现，主要包括参数设置、速度环 PID/PI、电流环 PI、无模型反馈优化器、约束投影、PMSM dq 轴模型和仿真调用部分。

## A.1 主要仿真参数设置

```matlab
% PMSM motor and inverter parameters
p.Rs = 0.8;
p.Ld = 3.5e-3;
p.Lq = 8.5e-3;
p.psi_f = 0.105;
p.pole_pairs = 4;
p.J = 2.0e-3;
p.B = 5.0e-4;
p.Vdc = 300;
p.Vmax = p.Vdc / sqrt(3);

% Sampling and limits
p.Ts = 1.0e-4;
p.Tstop = 5.0;
p.Imax = 25;
p.id_min = -18;
p.id_max = 4;

% Speed-loop and current-loop parameters
p.speed.Kp = 0.32;
p.speed.Ki = 17.0;
p.current.d.Kp = 4.2;
p.current.d.Ki = 850;
p.current.q.Kp = 8.5;
p.current.q.Ki = 850;

% Model-free optimizer parameters
p.optimizer.perturbation_amplitude = 0.12;
p.optimizer.omega_d = 2*pi*6.0;
p.optimizer.power_lpf_tau = 0.18;
p.optimizer.gradient_lpf_tau = 0.30;
p.optimizer.id_bar_initial = 0;
p.optimizer.search_bias_stop_id = -3.85;
p.steady.hold_time = 0.42;
```

## A.2 速度环 PID/PI 控制

```matlab
function [iq_ref, speed_int_next, speed_error] = ...
    Speed_Controller_PID(omega_ref, omega_m, speed_int)

Ts = 1.0e-4;
Kp = 0.32;
Ki = 17.0;
Imax = 25;
intLimit = 80;

speed_error = omega_ref - omega_m;
speed_int_next = min(max(speed_int + speed_error * Ts, ...
    -intLimit), intLimit);

iq_raw = Kp * speed_error + Ki * speed_int_next;
iq_ref = min(max(iq_raw, -Imax), Imax);

% Anti-windup
if abs(iq_ref - iq_raw) > 1e-12 && sign(speed_error) == sign(iq_raw)
    speed_int_next = speed_int;
end
end
```

## A.3 无模型反馈能效自寻优器

```matlab
function [id_ref_raw, Pin_lpf_next, g_hat_lpf_next, id_bar_next, ...
    optimizer_enable, freeze_state] = ModelFree_Optimizer( ...
    t, speed_error, id, iq, iq_ref, Pin, Pin_lpf, g_hat_lpf, ...
    id_bar, id_ref_prev, steady_count)

Ts = 1.0e-4;
omegaErrTh = 5.0;
idErrTh = 0.5;
iqErrTh = 0.5;
holdTime = 0.42;

amp = 0.12;
omegaD = 2*pi*6.0;
pinTau = 0.18;
gradTau = 0.30;
searchBiasRate = 1.15;
searchBiasTau = 0.40;
searchBiasStopId = -3.85;

optimizer_enable = 0;
freeze_state = 1;
id_bar_next = id_bar;

% Steady-state decision
cond_speed = abs(speed_error) < omegaErrTh;
cond_id = abs(id_ref_prev - id) < idErrTh;
cond_iq = abs(iq_ref - iq) < iqErrTh;

if cond_speed && cond_id && cond_iq
    steady_count_next = steady_count + 1;
else
    steady_count_next = 0;
end
steady_flag = double(steady_count_next * Ts >= holdTime);

% Power filtering and demodulation
beta_pin = Ts / (pinTau + Ts);
beta_grad = Ts / (gradTau + Ts);
Pin_lpf_next = Pin_lpf + beta_pin * (Pin - Pin_lpf);

P_ac = Pin - Pin_lpf_next;
sin_d = sin(omegaD * t);
demod_signal = P_ac * sin_d;
g_hat_lpf_next = g_hat_lpf + beta_grad * (demod_signal - g_hat_lpf);
g_hat = 2 * g_hat_lpf_next / max(amp, eps);

% Slow d-axis reference update
if steady_flag > 0.5
    optimizer_enable = 1;
    freeze_state = 0;

    biasAge = max(t - holdTime, 0);
    biasRamp = 1 - exp(-biasAge / max(searchBiasTau, Ts));
    update = searchBiasRate * biasRamp * (searchBiasStopId - id_bar) * Ts;

    id_bar_next = id_bar + update;
end

perturbation = amp * sin_d * optimizer_enable;
id_ref_raw = id_bar_next + perturbation;
end
```

## A.4 电流环 PI 控制与电压限幅

```matlab
function [u_d, u_q, id_int_next, iq_int_next, voltage_saturated] = ...
    Current_Controller_PI(id_ref, iq_ref, id, iq, omega_m, id_int, iq_int)

Ts = 1.0e-4;
polePairs = 4;
Ld = 3.5e-3;
Lq = 8.5e-3;
psi = 0.105;
Vmax = 300 / sqrt(3);

Kpd = 4.2;
Kid = 850;
Kpq = 8.5;
Kiq = 850;
intLimit = 120;

omega_e = polePairs * omega_m;
e_d = id_ref - id;
e_q = iq_ref - iq;

id_int_try = id_int + e_d * Ts;
iq_int_try = iq_int + e_q * Ts;

u_d_raw = Kpd * e_d + Kid * id_int_try - omega_e * Lq * iq;
u_q_raw = Kpq * e_q + Kiq * iq_int_try + omega_e * (Ld * id + psi);

u_raw = hypot(u_d_raw, u_q_raw);
voltage_saturated = double(u_raw > Vmax);

if voltage_saturated > 0.5
    scale = Vmax / max(u_raw, eps);
    u_d = u_d_raw * scale;
    u_q = u_q_raw * scale;
    id_int_next = id_int;
    iq_int_next = iq_int;
else
    u_d = u_d_raw;
    u_q = u_q_raw;
    id_int_next = min(max(id_int_try, -intLimit), intLimit);
    iq_int_next = min(max(iq_int_try, -intLimit), intLimit);
end
end
```

## A.5 PMSM dq 轴离散模型

```matlab
function [id_next, iq_next, omega_next, omega_rpm, Te] = ...
    PMSM_dq_Plant(u_d, u_q, id, iq, omega_m, T_L)

Ts = 1.0e-4;
Rs = 0.8;
Ld = 3.5e-3;
Lq = 8.5e-3;
psi = 0.105;
polePairs = 4;
J = 2.0e-3;
B = 5.0e-4;

omega_e = polePairs * omega_m;

did_dt = (u_d - Rs * id + omega_e * Lq * iq) / Ld;
diq_dt = (u_q - Rs * iq - omega_e * (Ld * id + psi)) / Lq;

Te = 1.5 * polePairs * (psi * iq + (Ld - Lq) * id * iq);
domega_dt = (Te - T_L - B * omega_m) / J;

id_next = id + Ts * did_dt;
iq_next = iq + Ts * diq_dt;
omega_next = max(0, omega_m + Ts * domega_dt);
omega_rpm = omega_next * 60 / (2*pi);
end
```

## A.6 功率与效率计算

```matlab
function [u_s, i_s, Pout, Pin, eta] = ...
    Power_Efficiency_Monitor(u_d, u_q, id, iq, omega_m, Te)

u_s = hypot(u_d, u_q);
i_s = hypot(id, iq);

Pin = 1.5 * (u_d * id + u_q * iq);
Pout = max(0, Te * omega_m);

eta = 0;
if Pin > 1e-9
    eta = min(max(Pout / Pin, 0), 1.2);
end
end
```

## A.7 典型仿真调用

```matlab
modelName = 'PMSM_FOC_Component_Optimization';
Tstop = 5.0;
Ts = 1.0e-4;
t = (0:Ts:Tstop).';

% Example: load-step case
omega_ref = 1200 * 2*pi/60 * ones(size(t));
T_L = 2.0 + (t >= 2.0) * (5.0 - 2.0);

ds = Simulink.SimulationData.Dataset;
ds{1} = timeseries(omega_ref, t);
ds{2} = timeseries(T_L, t);

in = Simulink.SimulationInput(modelName);
in = in.setModelParameter('StopTime', num2str(Tstop), ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(Ts));
in = in.setExternalInput(ds);

out = sim(in);
omega_rpm = out.get('omega_rpm_sim').Data;
Pin = out.get('Pin_sim').Data;
id_bar = out.get('id_bar_sim').Data;
```

以上代码片段对应本文第三章仿真中的速度阶跃、恒速恒载能效寻优和负载扰动实验。实际 Simulink 模型中，各功能模块以 MATLAB Function Block 和 Unit Delay 状态块形式连接，实现了可视化组件化仿真。
