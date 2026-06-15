function [stateNext, sig] = PMSM_Model(state, voltage, T_L, plant, Ts)
%PMSM_MODEL One fixed-step dq-axis IPMSM model update.

id = state.id;
iq = state.iq;
omega_m = state.omega_m;
omega_e = plant.p * omega_m;

u_d = voltage.u_d;
u_q = voltage.u_q;

did = (u_d - plant.R_s * id + omega_e * plant.L_q * iq) / plant.L_d;
diq = (u_q - plant.R_s * iq - omega_e * plant.L_d * id - omega_e * plant.psi_f) / plant.L_q;
T_e = 1.5 * plant.p * (plant.psi_f * iq + (plant.L_d - plant.L_q) * id * iq);
domega = (T_e - T_L - plant.B * omega_m) / plant.J_m;

stateNext.id = id + Ts * did;
stateNext.iq = iq + Ts * diq;
stateNext.omega_m = max(0, omega_m + Ts * domega);

sig.omega_e = omega_e;
sig.T_e = T_e;
sig.i_s = hypot(id, iq);
sig.u_s = hypot(u_d, u_q);
sig.P_in = 1.5 * (u_d * id + u_q * iq);
sig.P_out = max(0, T_e * omega_m);
sig.P_cu = 1.5 * plant.R_s * (id^2 + iq^2);
if sig.P_in > 1e-9
    sig.eta = max(0, min(1, sig.P_out / sig.P_in));
else
    sig.eta = 0;
end
end
