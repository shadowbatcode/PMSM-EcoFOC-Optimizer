if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end

baselineModel = fullfile(p.paths.models, 'PMSM_FOC_Component_Baseline.slx');
optimizationModel = fullfile(p.paths.models, 'PMSM_FOC_Component_Optimization.slx');
if exist(baselineModel, 'file') ~= 2 || exist(optimizationModel, 'file') ~= 2
    run(fullfile(p.paths.scripts, 'build_component_models.m'));
end

scenario.name = 'component_efficiency_optimization';
scenario.Tstop = p.Tstop;
scenario.omega_ref = @(t) p.exp.constant.omega_rpm * 2*pi/60;
scenario.load_torque = @(t) p.exp.constant.TL;

result.baseline = run_component_case('PMSM_FOC_Component_Baseline', scenario, p);
result.optimization = run_component_case('PMSM_FOC_Component_Optimization', scenario, p);
result.scenario = scenario;

summary = component_summary(result, p);
disp(summary);

save(fullfile(p.paths.data, 'component_efficiency_optimization.mat'), 'result', 'summary', 'scenario', 'p');
writetable(summary, fullfile(p.paths.tables, 'component_efficiency_optimization.csv'));

function caseRun = run_component_case(modelName, scenario, p)
load_system(fullfile(p.paths.models, [modelName '.slx']));
t = (0:p.Ts:scenario.Tstop).';
omegaRef = zeros(size(t));
loadTorque = zeros(size(t));
for k = 1:numel(t)
    omegaRef(k) = scenario.omega_ref(t(k));
    loadTorque(k) = scenario.load_torque(t(k));
end

ds = Simulink.SimulationData.Dataset;
ds{1} = timeseries(omegaRef, t);
ds{2} = timeseries(loadTorque, t);

in = Simulink.SimulationInput(modelName);
in = in.setModelParameter('StopTime', num2str(scenario.Tstop), ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(p.Ts));
in = in.setExternalInput(ds);
out = sim(in);

caseRun = struct();
caseRun.t = t;
names = pmsm_foc_output_names();
for k = 1:numel(names)
    ts = out.get([names{k} '_sim']);
    caseRun.(names{k}) = ts.Data(:);
end
caseRun.omega_ref = omegaRef;
caseRun.T_L = loadTorque;
close_system(modelName, 0);
end

function summary = component_summary(result, p)
methods = {'baseline'; 'optimization'};
rows = table();
for k = 1:numel(methods)
    method = methods{k};
    run = result.(method);
    tail = run.t >= run.t(end) * (1 - p.metric_tail_fraction);
    row = table(string(method), mean(run.Pin(tail)), mean(run.is(tail)), ...
        mean(run.eta(tail)) * 100, mean(run.omega_rpm(tail)), mean(run.id_bar(tail)), ...
        'VariableNames', {'method','mean_Pin_W','mean_is_A','efficiency_pct','mean_speed_rpm','mean_id_bar_A'});
    rows = [rows; row]; %#ok<AGROW>
end
basePin = rows.mean_Pin_W(strcmp(rows.method, 'baseline'));
baseIs = rows.mean_is_A(strcmp(rows.method, 'baseline'));
rows.pin_reduction_vs_id0_pct = 100 * (basePin - rows.mean_Pin_W) ./ max(abs(basePin), eps);
rows.current_reduction_vs_id0_pct = 100 * (baseIs - rows.mean_is_A) ./ max(abs(baseIs), eps);
summary = rows;
end
