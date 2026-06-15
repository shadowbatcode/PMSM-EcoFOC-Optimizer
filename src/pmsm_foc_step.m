function [x, y] = pmsm_foc_step(x, p, omega_ref, T_L, method, Ts)
%PMSM_FOC_STEP One fixed-step PMSM FOC update with optional MFO.
% This function is shared by the MATLAB experiments and the Simulink
% S-Function model so that both paths execute the same control algorithm.

omega_e = p.pole_pairs * x.omega_m;

speed_error = omega_ref - x.omega_m;
x.speed_int = clamp(x.speed_int + speed_error * Ts, -p.speed_int_limit, p.speed_int_limit);
iq_raw = p.speed.Kp * speed_error + p.speed.Ki * x.speed_int;
iq_ref = clamp(iq_raw, -p.Imax, p.Imax);
if abs(iq_ref - iq_raw) > 1e-12 && sign(speed_error) == sign(iq_raw)
    x.speed_int = x.speed_int - speed_error * Ts;
end

id_bar_before = x.id_bar;
perturbation = 0;
P_ac = x.Pin - x.Pin_lpf;
demod_signal = 0;
g_hat = x.g_hat_lpf;
steady_flag = 0;
optimizer_enable = 0;
projection_active = 0;
freeze_state = 1;

if strcmp(method, 'optimization')
    dPin_dt = (x.Pin - x.Pin_prev) / max(Ts, eps);
    cond_speed = abs(speed_error) < p.steady.omega_error_threshold;
    cond_id = abs(x.id_ref_prev - x.id) < p.steady.id_error_threshold;
    cond_iq = abs(iq_ref - x.iq) < p.steady.iq_error_threshold;
    cond_power = abs(dPin_dt) < p.steady.power_slope_threshold;
    cond_sat = ~(x.current_saturated || x.voltage_saturated);
    if cond_speed && cond_id && cond_iq && cond_power && cond_sat
        x.steady_count = x.steady_count + 1;
    else
        x.steady_count = 0;
    end
    steady_flag = double(x.steady_count * Ts >= p.steady.hold_time);

    beta_pin = Ts / (p.optimizer.power_lpf_tau + Ts);
    beta_grad = Ts / (p.optimizer.gradient_lpf_tau + Ts);
    x.Pin_lpf = x.Pin_lpf + beta_pin * (x.Pin - x.Pin_lpf);
    P_ac = x.Pin - x.Pin_lpf;
    sin_d = sin(p.optimizer.omega_d * x.t);
    demod_signal = P_ac * sin_d;
    x.g_hat_lpf = x.g_hat_lpf + beta_grad * (demod_signal - x.g_hat_lpf);
    g_hat = 2 * x.g_hat_lpf / max(p.optimizer.perturbation_amplitude, eps);

    if steady_flag > 0.5
        optimizer_enable = 1;
        freeze_state = 0;
        update = -p.optimizer.alpha * g_hat * p.optimizer.update_period;
        update = clamp(update, -p.optimizer.max_id_bar_step, p.optimizer.max_id_bar_step);
        id_bar_candidate = x.id_bar + update;
        [id_bar_projected, proj] = project_id_reference(id_bar_candidate, iq_ref, x.u_s, p);
        projection_active = double(proj.active);
        x.id_bar = slew_limit(id_bar_projected, x.id_bar, p.optimizer.id_rate_limit, Ts);
        x.perturbation_ramp = min(1, x.perturbation_ramp + Ts / max(p.optimizer.resume_ramp_time, Ts));
        perturbation = x.perturbation_ramp * p.optimizer.perturbation_amplitude * sin_d;
    else
        x.perturbation_ramp = max(0, x.perturbation_ramp - Ts / max(p.optimizer.freeze_ramp_time, Ts));
        perturbation = x.perturbation_ramp * p.optimizer.perturbation_amplitude * sin(p.optimizer.omega_d * x.t);
    end
    id_ref_raw = x.id_bar + perturbation;
elseif strcmp(method, 'baseline')
    id_ref_raw = 0;
else
    error('Unknown method: %s', method);
end

[id_ref, proj_final] = project_id_reference(id_ref_raw, iq_ref, x.u_s, p);
projection_active = double(projection_active || proj_final.active);

iq_limit = sqrt(max(p.Imax^2 - id_ref^2, 0));
iq_ref = clamp(iq_ref, -iq_limit, iq_limit);

e_d = id_ref - x.id;
e_q = iq_ref - x.iq;
id_int_try = x.id_int + e_d * Ts;
iq_int_try = x.iq_int + e_q * Ts;

u_d_raw = p.current.d.Kp * e_d + p.current.d.Ki * id_int_try ...
    - omega_e * p.Lq * x.iq;
u_q_raw = p.current.q.Kp * e_q + p.current.q.Ki * iq_int_try ...
    + omega_e * (p.Ld * x.id + p.psi_f);
u_raw = hypot(u_d_raw, u_q_raw);
voltage_saturated = u_raw > p.Vmax;
if voltage_saturated
    scale = p.Vmax / max(u_raw, eps);
    u_d = u_d_raw * scale;
    u_q = u_q_raw * scale;
else
    u_d = u_d_raw;
    u_q = u_q_raw;
    x.id_int = clamp(id_int_try, -p.current_int_limit, p.current_int_limit);
    x.iq_int = clamp(iq_int_try, -p.current_int_limit, p.current_int_limit);
end

did_dt = (u_d - p.Rs * x.id + omega_e * p.Lq * x.iq) / p.Ld;
diq_dt = (u_q - p.Rs * x.iq - omega_e * (p.Ld * x.id + p.psi_f)) / p.Lq;
Te = 1.5 * p.pole_pairs * (p.psi_f * x.iq + (p.Ld - p.Lq) * x.id * x.iq);
domega_dt = (Te - T_L - p.B * x.omega_m) / p.J;

x.id = x.id + Ts * did_dt;
x.iq = x.iq + Ts * diq_dt;
x.omega_m = max(0, x.omega_m + Ts * domega_dt);
x.t = x.t + Ts;

u_s = hypot(u_d, u_q);
i_s = hypot(x.id, x.iq);
Pin = 1.5 * (u_d * x.id + u_q * x.iq);
Pout = max(0, Te * x.omega_m);
eta = 0;
if Pin > 1e-9
    eta = clamp(Pout / Pin, 0, 1.2);
end

current_saturated = i_s > p.Imax * 0.999 || abs(iq_ref) > iq_limit * 0.999;
x.Pin_prev = x.Pin;
x.Pin = Pin;
x.u_s = u_s;
x.id_ref_prev = id_ref;
x.current_saturated = current_saturated;
x.voltage_saturated = voltage_saturated;

y.t = x.t;
y.omega_ref = omega_ref;
y.omega_m = x.omega_m;
y.omega_rpm = x.omega_m * 60 / (2*pi);
y.T_L = T_L;
y.id = x.id;
y.iq = x.iq;
y.id_ref = id_ref;
y.iq_ref = iq_ref;
y.ud = u_d;
y.uq = u_q;
y.us = u_s;
y.is = i_s;
y.Te = Te;
y.Pin = Pin;
y.Pin_lpf = x.Pin_lpf;
y.Pout = Pout;
y.eta = eta;
y.speed_error = speed_error;
y.perturbation = perturbation;
y.P_ac = P_ac;
y.demod_signal = demod_signal;
y.g_hat = g_hat;
y.id_bar = x.id_bar;
y.id_bar_before = id_bar_before;
y.optimizer_enable = optimizer_enable;
y.projection_active = projection_active;
y.freeze_state = freeze_state;
y.steady_flag = steady_flag;
y.current_saturated = double(current_saturated);
y.voltage_saturated = double(voltage_saturated);
y.dPin_dt = (Pin - x.Pin_prev) / max(Ts, eps);
end

function [id_projected, info] = project_id_reference(id_candidate, iq_ref, u_s_est, p)
id_projected = clamp(id_candidate, p.id_min, p.id_max);
box_active = abs(id_projected - id_candidate) > 1e-12;

id_limit = sqrt(max(p.Imax^2 - iq_ref^2, 0));
id_after_current = clamp(id_projected, -id_limit, id_limit);
current_active = abs(id_after_current - id_projected) > 1e-12;
id_projected = clamp(id_after_current, p.id_min, p.id_max);

voltage_active = u_s_est > p.Vmax * 0.995;
if voltage_active
    id_projected = max(p.id_min, id_projected - p.optimizer.voltage_projection_step);
end

id_projected = clamp(id_projected, p.id_min, p.id_max);
info.active = box_active || current_active || voltage_active;
end

function y = slew_limit(target, current, rate, Ts)
delta = clamp(target - current, -rate * Ts, rate * Ts);
y = current + delta;
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end
