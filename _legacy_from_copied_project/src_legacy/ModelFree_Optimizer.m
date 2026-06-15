function [id_ref, state, info] = ModelFree_Optimizer(input, state, p, Ts)
%MODELFREE_OPTIMIZER Extremum-seeking style d-axis current optimizer.

beta = Ts / (p.lpf_tau + Ts);
betaDemod = Ts / (p.demod_tau + Ts);
sin_p = sin(p.omega_p * input.t);

state.lpf_power = state.lpf_power + beta * (input.P_in - state.lpf_power);
power_ac = input.P_in - state.lpf_power;
state.lpf_demod = state.lpf_demod + betaDemod * (power_ac * sin_p - state.lpf_demod);
g_hat = (2 / max(p.a, eps)) * state.lpf_demod;
g_update = min(max(g_hat, -p.g_hat_limit), p.g_hat_limit);

id_candidate = state.id_bar;
if input.steady
    step = -p.alpha * Ts * g_update;
    step = min(max(step, -p.id_bar_step_max), p.id_bar_step_max);
    id_candidate = state.id_bar + step;
end

[id_projected, proj] = Projection_Omega(id_candidate, input.iq_ref, input.u_s_est, p);
if input.steady
    state.id_bar = id_projected;
    if input.t > p.best_hold_time
        if ~state.best_initialized
            state.best_initialized = true;
            state.best_power = state.lpf_power;
            state.best_id_bar = state.id_bar;
        elseif state.lpf_power < state.best_power
            state.best_power = state.lpf_power;
            state.best_id_bar = state.id_bar;
        elseif state.lpf_power > state.best_power + p.power_backtrack_margin
            state.id_bar = state.id_bar + p.power_backtrack_gain * (state.best_id_bar - state.id_bar);
        end
    end
    state.id_bar_applied = state.id_bar_applied + p.opt_smooth * (state.id_bar - state.id_bar_applied);
    perturb = p.a * sin_p;
else
    perturb = 0;
end

id_ref = state.id_bar_applied + perturb;
id_ref = min(max(id_ref, p.i_d_min), p.i_d_max);

info.g_hat = g_hat;
info.g_update = g_update;
info.Pbar = state.lpf_power;
info.id_bar_candidate = id_candidate;
info.id_bar_projected = id_projected;
info.id_bar = state.id_bar;
info.id_bar_applied = state.id_bar_applied;
info.perturb = perturb;
info.optimizer_active = input.steady;
info.projection = proj;
info.current_constraint = proj.current_limited;
info.voltage_constraint = proj.voltage_limited;
info.constraint_limited = proj.constraint_limited;
end
