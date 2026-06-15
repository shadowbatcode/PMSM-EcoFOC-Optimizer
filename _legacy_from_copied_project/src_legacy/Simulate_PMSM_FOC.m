function run = Simulate_PMSM_FOC(method, scenario, p)
%SIMULATE_PMSM_FOC Run one closed-loop PMSM FOC simulation case.

if isfield(scenario, 'seed')
    rng(p.random_seed + scenario.seed);
else
    rng(p.random_seed);
end

Tsim = getFieldOrDefault(scenario, 'Tsim', p.Tsim);
Ts = getFieldOrDefault(scenario, 'Ts', p.Ts);
t = (0:Ts:Tsim).';
N = numel(t);

plant = makePlantParameters(p, scenario);
ctrl = p;

motor.id = getFieldOrDefault(scenario, 'id0', 0);
motor.iq = getFieldOrDefault(scenario, 'iq0', 0);
motor.omega_m = getFieldOrDefault(scenario, 'omega0', 0);

curState.int_d = 0;
curState.int_q = 0;
spdState.int_w = 0;
spdState.prev_e_w = 0;
steadyState.counter = 0;
steadyState.prev_Pbar = 0;
optState.id_bar = 0;
optState.id_bar_applied = 0;
optState.lpf_power = 0;
optState.lpf_demod = 0;
optState.best_initialized = false;
optState.best_power = Inf;
optState.best_id_bar = 0;

delayState.id = motor.id;
delayState.iq = motor.iq;
delayState.omega_m = motor.omega_m;
lastPin = 0;
lastUs = 0;
pbar = 0;
betaP = Ts / (p.lpf_tau + Ts);

run = initializeLog(N, method, scenario);

for k = 1:N
    tk = t(k);
    omega_ref = scenario.omega_ref(tk);
    T_L = scenario.load_torque(tk);

    ctrl = updateInverterLimit(p, scenario, tk);

    meas = makeMeasurement(motor, delayState, plant, scenario, p, Ts);
    delayState = updateDelayState(delayState, motor, scenario, p, Ts);

    [iq_ref, spdState, spdInfo] = Speed_Controller_PID(omega_ref, meas.omega_m, spdState, ctrl, Ts);
    id_ref = 0;
    optInfo = emptyOptimizerInfo();
    steadyFlag = false;

    if strcmp(method, 'mtpa')
        id_ref = MTPA_Controller(iq_ref, p);
    elseif strcmp(method, 'mfo')
        [steadyState, steadyFlag] = SteadyState_Detector(omega_ref, meas.omega_m, iq_ref, meas.iq, optState.lpf_power, steadyState, p);
        optInput.t = tk;
        optInput.P_in = lastPin;
        optInput.iq_ref = iq_ref;
        optInput.u_s_est = lastUs;
        optInput.steady = steadyFlag;
        [id_ref, optState, optInfo] = ModelFree_Optimizer(optInput, optState, ctrl, Ts);
    end

    iqLimit = min(ctrl.i_q_max, sqrt(max(ctrl.i_s_max^2 - id_ref^2, 0)));
    iq_ref = min(max(iq_ref, -iqLimit), iqLimit);

    [voltage, curState, curInfo] = Current_Controller_PI(id_ref, iq_ref, meas, curState, ctrl, Ts);
    [motorNext, sig] = PMSM_Model(motor, voltage, T_L, plant, Ts);

    if ~strcmp(method, 'mfo')
        pbar = pbar + betaP * (sig.P_in - pbar);
        optInfo.Pbar = pbar;
    end

    run.t(k) = tk;
    run.omega_ref(k) = omega_ref;
    run.omega_m(k) = motor.omega_m;
    run.omega_rpm(k) = motor.omega_m * p.rad_to_rpm;
    run.omega_ref_rpm(k) = omega_ref * p.rad_to_rpm;
    run.T_L(k) = T_L;
    run.id(k) = motor.id;
    run.iq(k) = motor.iq;
    run.id_ref(k) = id_ref;
    run.iq_ref(k) = iq_ref;
    run.u_d(k) = voltage.u_d;
    run.u_q(k) = voltage.u_q;
    run.u_s(k) = sig.u_s;
    run.u_s_raw(k) = curInfo.u_s_raw;
    run.i_s(k) = sig.i_s;
    run.T_e(k) = sig.T_e;
    run.P_in(k) = sig.P_in;
    run.Pbar_in(k) = optInfo.Pbar;
    run.P_out(k) = sig.P_out;
    run.P_cu(k) = sig.P_cu;
    run.eta(k) = sig.eta;
    run.g_hat(k) = optInfo.g_hat;
    run.id_bar(k) = optInfo.id_bar;
    run.id_bar_candidate(k) = optInfo.id_bar_candidate;
    run.id_bar_projected(k) = optInfo.id_bar_projected;
    run.steady_flag(k) = steadyFlag;
    run.optimizer_active(k) = optInfo.optimizer_active;
    run.current_constraint(k) = optInfo.current_constraint;
    run.voltage_constraint(k) = optInfo.voltage_constraint || curInfo.voltage_limited;
    run.constraint_flag(k) = run.current_constraint(k) || run.voltage_constraint(k);
    run.speed_error(k) = spdInfo.e_w;
    run.U_dc(k) = ctrl.U_dc;

    lastPin = sig.P_in;
    lastUs = sig.u_s;
    motor = motorNext;
end
end

function plant = makePlantParameters(p, scenario)
plant = p;
if isfield(scenario, 'perturbed') && scenario.perturbed
    plant.R_s = p.R_s * p.perturb.R_s_scale;
    plant.psi_f = p.psi_f * p.perturb.psi_f_scale;
    plant.L_d = p.L_d * p.perturb.L_d_scale;
    plant.L_q = p.L_q * p.perturb.L_q_scale;
end
end

function ctrl = updateInverterLimit(p, scenario, t)
ctrl = p;
if isfield(scenario, 'nonideal') && scenario.nonideal
    ripple = p.noise.udc_ripple_amp * sin(2*pi*p.noise.udc_ripple_freq*t);
    ctrl.U_dc = p.U_dc * (1 + ripple);
    ctrl.u_s_max = ctrl.U_dc / sqrt(3);
end
end

function meas = makeMeasurement(motor, delayState, plant, scenario, p, Ts)
useDelay = isfield(scenario, 'nonideal') && scenario.nonideal;
if useDelay
    src = delayState;
else
    src = motor;
end

meas.id = src.id;
meas.iq = src.iq;
meas.omega_m = src.omega_m;
if useDelay
    meas.id = meas.id + p.noise.current_std * randn();
    meas.iq = meas.iq + p.noise.current_std * randn();
    meas.omega_m = meas.omega_m + p.noise.speed_std * randn();
end
meas.omega_e = plant.p * meas.omega_m;
meas.Ts = Ts;
end

function delayState = updateDelayState(delayState, motor, scenario, p, Ts)
if isfield(scenario, 'nonideal') && scenario.nonideal
    beta = Ts / (p.noise.delay_time + Ts);
    delayState.id = delayState.id + beta * (motor.id - delayState.id);
    delayState.iq = delayState.iq + beta * (motor.iq - delayState.iq);
    delayState.omega_m = delayState.omega_m + beta * (motor.omega_m - delayState.omega_m);
else
    delayState = motor;
end
end

function value = getFieldOrDefault(s, fieldName, defaultValue)
if isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function info = emptyOptimizerInfo()
info.g_hat = 0;
info.Pbar = 0;
info.id_bar = 0;
info.id_bar_candidate = 0;
info.id_bar_projected = 0;
info.optimizer_active = false;
info.current_constraint = false;
info.voltage_constraint = false;
end

function run = initializeLog(N, method, scenario)
run.method = method;
run.case_name = scenario.name;
run.description = scenario.description;
fields = {'t','omega_ref','omega_m','omega_rpm','omega_ref_rpm','T_L','id','iq', ...
    'id_ref','iq_ref','u_d','u_q','u_s','u_s_raw','i_s','T_e','P_in','Pbar_in', ...
    'P_out','P_cu','eta','g_hat','id_bar','id_bar_candidate','id_bar_projected', ...
    'steady_flag','optimizer_active','current_constraint','voltage_constraint', ...
    'constraint_flag','speed_error','U_dc'};
for i = 1:numel(fields)
    run.(fields{i}) = zeros(N, 1);
end
end
