function [iq_ref, state, info] = Speed_Controller_PID(omega_ref, omega_meas, state, ctrl, Ts)
%SPEED_CONTROLLER_PID Speed PID controller with q-axis current saturation.

e_w = omega_ref - omega_meas;
int_candidate = state.int_w + e_w * Ts;
derivative = (e_w - state.prev_e_w) / Ts;

raw = ctrl.K_pw * e_w + ctrl.K_iw * int_candidate + ctrl.K_dw * derivative;
iq_ref = min(max(raw, -ctrl.i_q_max), ctrl.i_q_max);

saturated = abs(raw - iq_ref) > 1e-12;
drives_outward = saturated && (sign(e_w) == sign(raw - iq_ref));
if ~drives_outward
    state.int_w = int_candidate;
end
state.prev_e_w = e_w;

info.e_w = e_w;
info.raw_iq_ref = raw;
info.saturated = saturated;
end
