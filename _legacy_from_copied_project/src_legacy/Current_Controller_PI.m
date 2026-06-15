function [voltage, state, info] = Current_Controller_PI(id_ref, iq_ref, meas, state, ctrl, Ts)
%CURRENT_CONTROLLER_PI d/q current PI controller with decoupling and limits.

e_d = id_ref - meas.id;
e_q = iq_ref - meas.iq;

int_d_candidate = state.int_d + e_d * Ts;
int_q_candidate = state.int_q + e_q * Ts;

u_d_pi = ctrl.K_pd * e_d + ctrl.K_id * int_d_candidate;
u_q_pi = ctrl.K_pq * e_q + ctrl.K_iq * int_q_candidate;

u_d_raw = u_d_pi - meas.omega_e * ctrl.L_q * meas.iq;
u_q_raw = u_q_pi + meas.omega_e * ctrl.L_d * meas.id + meas.omega_e * ctrl.psi_f;

u_s_raw = hypot(u_d_raw, u_q_raw);
limitActive = u_s_raw > ctrl.u_s_max;
if limitActive
    scale = ctrl.u_s_max / max(u_s_raw, eps);
    u_d = u_d_raw * scale;
    u_q = u_q_raw * scale;
else
    u_d = u_d_raw;
    u_q = u_q_raw;
    state.int_d = int_d_candidate;
    state.int_q = int_q_candidate;
end

voltage.u_d = u_d;
voltage.u_q = u_q;
info.e_d = e_d;
info.e_q = e_q;
info.u_s_raw = u_s_raw;
info.u_s = hypot(u_d, u_q);
info.voltage_limited = limitActive;
end
