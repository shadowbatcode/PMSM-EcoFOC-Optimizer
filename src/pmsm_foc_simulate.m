function run = pmsm_foc_simulate(method, scenario, p)
%PMSM_FOC_SIMULATE Run one deterministic fixed-step PMSM FOC case.

Ts = p.Ts;
t = (0:Ts:scenario.Tstop).';
n = numel(t);
x = pmsm_initial_state(p);
run = init_log(n, method, scenario.name);

for k = 1:n
    tk = t(k);
    omega_ref = scenario.omega_ref(tk);
    T_L = scenario.load_torque(tk);
    [x, y] = pmsm_foc_step(x, p, omega_ref, T_L, method, Ts);
    run.t(k) = tk;
    names = fieldnames(y);
    for i = 1:numel(names)
        name = names{i};
        if isfield(run, name)
            run.(name)(k) = y.(name);
        end
    end
end
end

function x = pmsm_initial_state(p)
x.t = 0;
x.id = 0;
x.iq = 0;
x.omega_m = 0;
x.speed_int = 0;
x.id_int = 0;
x.iq_int = 0;
x.Pin = 0;
x.Pin_prev = 0;
x.Pin_lpf = 0;
x.g_hat_lpf = 0;
x.id_bar = p.optimizer.id_bar_initial;
x.u_s = 0;
x.id_ref_prev = 0;
x.steady_count = 0;
x.perturbation_ramp = 0;
x.current_saturated = false;
x.voltage_saturated = false;
end

function run = init_log(n, method, case_name)
run.method = method;
run.case_name = case_name;
fields = {'t','omega_ref','omega_m','omega_rpm','T_L','id','iq','id_ref','iq_ref', ...
    'ud','uq','us','is','Te','Pin','Pin_lpf','Pout','eta','speed_error', ...
    'perturbation','P_ac','demod_signal','g_hat','id_bar','id_bar_before', ...
    'optimizer_enable','projection_active','freeze_state','steady_flag', ...
    'current_saturated','voltage_saturated','dPin_dt'};
for i = 1:numel(fields)
    run.(fields{i}) = zeros(n, 1);
end
end
