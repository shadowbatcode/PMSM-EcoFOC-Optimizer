function [state, steadyFlag, info] = SteadyState_Detector(omega_ref, omega_meas, iq_ref, iq_meas, Pbar, state, p)
%STEADYSTATE_DETECTOR Detect quasi-steady operation before optimizer update.

deltaP = Pbar - state.prev_Pbar;
cond_w = abs(omega_ref - omega_meas) < p.epsilon_w;
cond_i = abs(iq_ref - iq_meas) < p.epsilon_i;
cond_P = abs(deltaP) < p.epsilon_P;

if cond_w && cond_i && cond_P
    state.counter = state.counter + 1;
else
    state.counter = 0;
end

steadyFlag = state.counter >= p.N_s;
state.prev_Pbar = Pbar;

info.cond_w = cond_w;
info.cond_i = cond_i;
info.cond_P = cond_P;
info.deltaP = deltaP;
info.counter = state.counter;
end
