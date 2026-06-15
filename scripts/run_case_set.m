function result = run_case_set(scenario, p, case_name)
%RUN_CASE_SET Run baseline and optimization cases through executable Simulink models.

result = struct();
for i = 1:numel(p.method_order)
    method = p.method_order{i};
    caseRun = run_simulink_case(method, scenario, p, case_name);
    result.(method) = caseRun;
end
result.scenario = scenario;
end

function caseRun = run_simulink_case(method, scenario, p, case_name)
modelName = model_for_method(method);
modelPath = fullfile(p.paths.models, [modelName '.slx']);
if exist(modelPath, 'file') ~= 2
    run(fullfile(p.paths.scripts, 'build_models.m'));
end

assignin('base', 'p', p);
assignin('base', 'scenario', scenario);

load_system(modelPath);
set_param(modelName, ...
    'StopTime', num2str(scenario.Tstop), ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(p.Ts), ...
    'ReturnWorkspaceOutputs', 'on');

out = sim(modelName, ...
    'StopTime', num2str(scenario.Tstop), ...
    'ReturnWorkspaceOutputs', 'on');

caseRun = simulation_output_to_run(out, method, case_name, scenario, p);

try
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
catch
end
end

function modelName = model_for_method(method)
switch method
    case 'baseline'
        modelName = 'PMSM_FOC_Baseline';
    case 'optimization'
        modelName = 'PMSM_FOC_Optimization';
    otherwise
        error('Unknown method: %s', method);
end
end

function caseRun = simulation_output_to_run(out, method, case_name, scenario, p)
names = pmsm_foc_output_names();
omega = get_timeseries(out, 'omega_m_sim');
t = omega.Time(:);
n = numel(t);

caseRun = init_run_log(n, method, case_name);
caseRun.source = 'simulink';
caseRun.t = t;
caseRun.method = method;
caseRun.case_name = case_name;

for k = 1:numel(names)
    name = names{k};
    ts = get_timeseries(out, [name '_sim']);
    data = ts.Data;
    caseRun.(name) = data(:);
end

for k = 1:n
    caseRun.omega_ref(k) = scenario.omega_ref(t(k));
    caseRun.T_L(k) = scenario.load_torque(t(k));
end

% Compatibility aliases expected by metric and plotting scripts.
caseRun.ud = zeros(n, 1);
caseRun.uq = zeros(n, 1);
caseRun.Pin_lpf = zeros(n, 1);
caseRun.P_ac = zeros(n, 1);
caseRun.demod_signal = zeros(n, 1);
caseRun.id_bar_before = caseRun.id_bar;
caseRun.dPin_dt = [0; diff(caseRun.Pin)] ./ max(p.Ts, eps);
caseRun.perturbation = caseRun.id_ref - caseRun.id_bar;
end

function ts = get_timeseries(out, name)
if ~isprop(out, name)
    error('Missing Simulink output signal: %s', name);
end
ts = out.get(name);
if ~isa(ts, 'timeseries')
    error('Expected timeseries for %s, got %s.', name, class(ts));
end
end

function caseRun = init_run_log(n, method, case_name)
caseRun.method = method;
caseRun.case_name = case_name;
fields = {'t','omega_ref','omega_m','omega_rpm','T_L','id','iq','id_ref','iq_ref', ...
    'ud','uq','us','is','Te','Pin','Pin_lpf','Pout','eta','speed_error', ...
    'perturbation','P_ac','demod_signal','g_hat','id_bar','id_bar_before', ...
    'optimizer_enable','projection_active','freeze_state','steady_flag', ...
    'current_saturated','voltage_saturated','dPin_dt'};
for i = 1:numel(fields)
    caseRun.(fields{i}) = zeros(n, 1);
end
end
