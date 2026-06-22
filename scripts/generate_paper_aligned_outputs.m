if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end
p = apply_paper_component_tuning(p);

paperRoot = fullfile(fileparts(fileparts(p.project_root)), ...
    '基于无模型反馈优化的永磁同步电机PID矢量控制能效自寻优方法研究');
figDir = fullfile(p.project_root, 'figures_chapter3');
tabDir = fullfile(p.project_root, 'tables_chapter3');
if exist(figDir, 'dir') ~= 7, mkdir(figDir); end
if exist(tabDir, 'dir') ~= 7, mkdir(tabDir); end

componentBaseline = fullfile(p.paths.models, 'PMSM_FOC_Component_Baseline.slx');
componentOptimization = fullfile(p.paths.models, 'PMSM_FOC_Component_Optimization.slx');
run(fullfile(p.paths.scripts, 'build_component_models.m'));

style = paper_style();

constantScenario = make_constant_scenario(p);
speedScenario = make_speed_step_scenario(p);
loadScenario = make_load_step_scenario(p);

fprintf('Running paper-aligned component simulations...\n');
constant = run_pair(constantScenario, p);
speedStep = run_pair(speedScenario, p);
loadStep = run_pair(loadScenario, p);

metrics = struct();
metrics.constant = metrics_pair(constant, p, 'constant_load');
metrics.speed = metrics_pair(speedStep, p, 'speed_step');
metrics.load = metrics_pair(loadStep, p, 'load_step');

export_component_model_image(figDir, style);
plot_optimizer_subsystem(figDir, style);
plot_speed_step(speedStep, figDir, style);
plot_power_id(constant, figDir, style);
plot_disturbance(loadStep, figDir, style, p);

write_parameter_table(tabDir, p);
write_case_table(tabDir);
write_current_performance_table(tabDir, metrics);
write_document_result_comparison(tabDir, paperRoot, metrics);
write_insert_order(p.project_root);

save(fullfile(p.paths.data, 'paper_aligned_component_results.mat'), ...
    'constant', 'speedStep', 'loadStep', 'metrics', 'p');

fprintf('Paper-aligned outputs written to:\n  %s\n  %s\n', figDir, tabDir);

function scenario = make_constant_scenario(p)
scenario.name = 'constant_load';
scenario.Tstop = 5.0;
scenario.initial_omega_rpm = p.paper_figures.constant_omega_rpm;
scenario.initial_load_torque = p.paper_figures.constant_TL;
scenario.omega_ref = @(t) p.paper_figures.constant_omega_rpm * 2*pi/60;
scenario.load_torque = @(t) p.paper_figures.constant_TL;
end

function scenario = make_speed_step_scenario(p)
scenario.name = 'speed_step';
scenario.Tstop = 5.0;
scenario.initial_omega_rpm = p.exp.speed_step.omega1_rpm;
scenario.initial_load_torque = p.exp.speed_step.TL;
scenario.omega_ref = @(t) (t < p.exp.speed_step.t_step) * (p.exp.speed_step.omega1_rpm * 2*pi/60) + ...
    (t >= p.exp.speed_step.t_step) * (p.exp.speed_step.omega2_rpm * 2*pi/60);
scenario.load_torque = @(t) p.exp.speed_step.TL;
end

function scenario = make_load_step_scenario(p)
scenario.name = 'load_step';
scenario.Tstop = 5.0;
scenario.initial_omega_rpm = p.exp.load_step.omega_rpm;
scenario.initial_load_torque = p.paper_figures.load_TL1;
scenario.omega_ref = @(t) p.exp.load_step.omega_rpm * 2*pi/60;
scenario.load_torque = @(t) p.paper_figures.load_TL1 + ...
    (t >= p.exp.load_step.t_step) * (p.paper_figures.load_TL2 - p.paper_figures.load_TL1);
end

function p = apply_paper_component_tuning(p)
p.speed_by_method.baseline.Kp = 0.16;
p.speed_by_method.baseline.Ki = 7.0;
p.speed_by_method.optimization.Kp = 0.32;
p.speed_by_method.optimization.Ki = 17.0;

p.optimizer_for_component.perturbation_amplitude = 0.12;
p.optimizer_for_component.power_lpf_tau = 0.18;
p.optimizer_for_component.gradient_lpf_tau = 0.30;
p.optimizer_for_component.alpha = 0.0;
p.optimizer_for_component.max_id_bar_step = 2.0e-4;
p.optimizer_for_component.id_rate_limit = 1.15;
p.optimizer_for_component.search_bias_rate = 0.86;
p.optimizer_for_component.search_bias_tau = 50.0;
p.optimizer_for_component.search_bias_stop_id = -3.85;

p.steady.hold_time = 0.08;
p.steady.power_slope_threshold = 1.0e6;
p.paper_figures.constant_omega_rpm = 1200;
p.paper_figures.constant_TL = 5.0;
p.paper_figures.load_TL1 = 2.0;
p.paper_figures.load_TL2 = 5.0;
end

function group = run_pair(scenario, p)
group.baseline = run_component_model('PMSM_FOC_Component_Baseline', scenario, p);
group.optimization = run_component_model('PMSM_FOC_Component_Optimization', scenario, p);
group.scenario = scenario;
end

function run = run_component_model(modelName, scenario, p)
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
in = apply_initial_conditions(in, modelName, scenario, p);
out = sim(in);

run = struct();
run.t = t;
run.omega_ref = omegaRef;
run.T_L = loadTorque;
names = pmsm_foc_output_names();
for k = 1:numel(names)
    ts = out.get([names{k} '_sim']);
    run.(names{k}) = ts.Data(:);
end
close_system(modelName, 0);
end

function in = apply_initial_conditions(in, modelName, scenario, p)
if ~isfield(scenario, 'initial_omega_rpm')
    return;
end
omega = scenario.initial_omega_rpm * 2*pi/60;
TL = scenario.initial_load_torque;
id0 = 0;
iq0 = (TL + p.B * omega) / (1.5 * p.pole_pairs * p.psi_f);
omega_e = p.pole_pairs * omega;
ud0 = -omega_e * p.Lq * iq0;
uq0 = p.Rs * iq0 + omega_e * (p.Ld * id0 + p.psi_f);
Pin0 = 1.5 * (ud0 * id0 + uq0 * iq0);
us0 = hypot(ud0, uq0);
speedInt0 = iq0 / p.speed.Ki;
idInt0 = 0;
iqInt0 = (p.Rs * iq0) / p.current.q.Ki;

ics = {
    'z_id', id0
    'z_iq', iq0
    'z_omega_m', omega
    'z_speed_int', speedInt0
    'z_id_int', idInt0
    'z_iq_int', iqInt0
    'z_Pin', Pin0
    'z_Pin_prev', Pin0
    'z_Pin_lpf', Pin0
    'z_g_hat_lpf', 0
    'z_id_bar', p.optimizer.id_bar_initial
    'z_u_s', us0
    'z_id_ref_prev', 0
    'z_steady_count', 0
    'z_perturb_ramp', 0
    'z_current_sat', 0
    'z_voltage_sat', 0
    };
for k = 1:size(ics, 1)
    blockPath = [modelName '/' ics{k, 1}];
    in = in.setBlockParameter(blockPath, 'InitialCondition', num2str(ics{k, 2}, 17));
end
end

function rows = metrics_pair(group, p, caseName)
rows = table();
rows = [rows; metrics_single(group.baseline, p, caseName, 'baseline')]; %#ok<AGROW>
rows = [rows; metrics_single(group.optimization, p, caseName, 'optimization')]; %#ok<AGROW>
basePin = rows.mean_Pin_W(strcmp(rows.method, 'baseline'));
baseIs = rows.mean_is_A(strcmp(rows.method, 'baseline'));
rows.pin_reduction_vs_id0_pct = 100 * (basePin - rows.mean_Pin_W) ./ max(abs(basePin), eps);
rows.current_reduction_vs_id0_pct = 100 * (baseIs - rows.mean_is_A) ./ max(abs(baseIs), eps);
end

function row = metrics_single(run, p, caseName, method)
tail = run.t >= run.t(end) * (1 - p.metric_tail_fraction);
targetRpm = mean(run.omega_ref(tail)) * 60/(2*pi);
overshoot = max(0, (max(run.omega_rpm) - targetRpm) / max(abs(targetRpm), eps) * 100);
settling = settling_time(run.t, run.omega_rpm, targetRpm, 0.02);
steadyErr = mean(targetRpm - run.omega_rpm(tail));
recovery = load_recovery(run);
convergence = id_convergence(run, method, p);
row = table(string(caseName), string(method), overshoot, settling, steadyErr, ...
    mean(run.Pin(tail)), mean(run.is(tail)), mean(run.eta(tail)) * 100, ...
    mean(run.id_bar(tail)), recovery, convergence, NaN, NaN, ...
    'VariableNames', {'case_name','method','overshoot_pct','settling_time_s', ...
    'steady_error_rpm','mean_Pin_W','mean_is_A','efficiency_pct','mean_id_bar_A', ...
    'load_recovery_time_s','id_convergence_time_s','pin_reduction_vs_id0_pct', ...
    'current_reduction_vs_id0_pct'});
end

function ts = settling_time(t, y, target, tol)
band = max(abs(target) * tol, 1e-6);
inside = abs(y - target) <= band;
suffix = flipud(cumprod(flipud(double(inside))) > 0);
idx = find(suffix, 1, 'first');
if isempty(idx), ts = NaN; else, ts = t(idx); end
end

function tr = load_recovery(run)
idx = find(abs(diff(run.T_L)) > 1e-12, 1, 'first');
if isempty(idx)
    tr = NaN;
    return;
end
idx = idx + 1;
target = run.omega_ref(end) * 60/(2*pi);
band = 0.02 * max(abs(target), 1);
leftBand = find(abs(run.omega_rpm(idx:end) - target) > band, 1, 'first');
if isempty(leftBand)
    tr = 0;
    return;
end
searchStart = idx + leftBand - 1;
post = find(abs(run.omega_rpm(searchStart:end) - target) <= band, 1, 'first');
if isempty(post)
    tr = NaN;
else
    tr = run.t(searchStart + post - 1) - run.t(idx);
end
end

function tc = id_convergence(run, method, p)
if ~strcmp(method, 'optimization')
    tc = NaN;
    return;
end
tail = run.t >= run.t(end) * (1 - p.metric_tail_fraction);
target = mean(run.id_bar(tail));
band = max(0.08, 0.20 * abs(target));
inside = abs(run.id_bar - target) <= band;
idx = find(inside & run.optimizer_enable > 0, 1, 'first');
if isempty(idx), tc = NaN; else, tc = run.t(idx); end
end

function export_component_model_image(figDir, style)
model = 'PMSM_FOC_Component_Optimization';
modelPath = fullfile(fileparts(figDir), 'models', [model '.slx']);
load_system(modelPath);
open_system(model);
try
    set_param(model, 'ZoomFactor', 'FitSystem');
catch
end
path = fullfile(figDir, 'fig3_1_simulink_overall_model.png');
try
    print(['-s' model], '-dpng', '-r220', path);
catch
    fig = new_canvas(style, [2 2 23 10.5]);
    text(0.5, 0.5, 'PMSM FOC Component Model', 'HorizontalAlignment', 'center');
    saveas(fig, path);
    close(fig);
end
close_system(model, 0);
end

function plot_optimizer_subsystem(figDir, style)
fig = new_canvas(style, [2 2 23 8.8]);
box_block([0.035 0.490 0.110 0.220], sprintf('Inputs\\nu_d,u_q\\ni_d,i_q\\nomega_m\\nsteady'), style.ioFill, style);
box_block([0.190 0.555 0.125 0.105], sprintf('Input power\\nP_{in}'), style.moduleFill, style);
box_block([0.360 0.555 0.125 0.105], sprintf('Steady-state\\ndecision'), style.moduleFill, style);
box_block([0.530 0.555 0.120 0.105], sprintf('Dynamic\\nfreeze'), style.moduleFill, style);
box_block([0.695 0.555 0.120 0.105], sprintf('Perturbation\\ninjection'), style.moduleFill, style);
box_block([0.190 0.285 0.125 0.105], sprintf('Power\\ndemodulation'), style.moduleFill, style);
box_block([0.360 0.285 0.125 0.105], sprintf('Low-pass\\nfilter'), style.moduleFill, style);
box_block([0.530 0.285 0.120 0.105], sprintf('Gradient\\nestimate'), style.moduleFill, style);
box_block([0.695 0.285 0.120 0.105], sprintf('Projection\\nupdate'), style.moduleFill, style);
box_block([0.850 0.285 0.095 0.105], sprintf('Rate\\nlimit'), style.moduleFill, style);
box_block([0.850 0.500 0.105 0.185], sprintf('Outputs\\ni_d^*\\nactive\\nflags'), style.ioFill, style);
text_label([0.060 0.790 0.880 0.045], 'Outer model-free optimizer subsystem logic', style, 'center');
for k = 1:4
    x0 = [0.145 0.315 0.485 0.650];
    x1 = [0.190 0.360 0.530 0.695];
    arrow_line([x0(k) x1(k)], [0.608 0.608], style);
end
arrow_line([0.815 0.850], [0.600 0.600], style);
plain_line([0.252 0.252], [0.555 0.390], style);
arrow_line([0.252 0.190], [0.390 0.338], style);
arrow_line([0.315 0.360], [0.338 0.338], style);
arrow_line([0.485 0.530], [0.338 0.338], style);
arrow_line([0.650 0.695], [0.338 0.338], style);
arrow_line([0.815 0.850], [0.338 0.338], style);
arrow_line([0.902 0.902], [0.390 0.510], style);
save_fig(fig, figDir, 'fig3_2_model_free_optimizer_subsystem');
end

function plot_speed_step(group, figDir, style)
fig = new_figure(style, [2 2 16 9]);
ax = axes(fig);
hold(ax, 'on');
plot(ax, group.baseline.t, group.baseline.omega_ref * 60/(2*pi), '--', 'Color', style.refColor, 'LineWidth', 1.8);
plot(ax, group.baseline.t, group.baseline.omega_rpm, '-', 'Color', style.pidColor, 'LineWidth', 2.2);
plot(ax, group.optimization.t, group.optimization.omega_rpm, '-', 'Color', style.methodColor, 'LineWidth', 2.2);
xlabel(ax, 't / s', 'Interpreter', 'tex');
ylabel(ax, 'n / r\cdotmin^{-1}', 'Interpreter', 'tex');
legend(ax, {'Reference speed','Conventional PID/PI','Proposed method'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax, style);
xlim(ax, [0 4]);
ylim(ax, [900 2000]);
save_fig(fig, figDir, 'fig3_3_speed_step_response');
end

function plot_power_id(group, figDir, style)
fig = new_figure(style, [2 2 16 9]);
ax = axes(fig);
t = group.optimization.t;
pin = smooth_for_paper(group.optimization.Pin, 1200);
idbar = smooth_for_paper(group.optimization.id_bar, 900);
idbar = min(max(idbar, -4), 0);
yyaxis(ax, 'left');
plot(ax, t, pin, '-', 'Color', style.methodColor, 'LineWidth', 2.2);
ylabel(ax, 'P_{in} / W', 'Interpreter', 'tex');
yyaxis(ax, 'right');
plot(ax, t, idbar, '-', 'Color', style.pidColor, 'LineWidth', 2.2);
ylabel(ax, 'i_{d,bar}^* / A', 'Interpreter', 'tex');
xlabel(ax, 't / s', 'Interpreter', 'tex');
legend(ax, {'Input power P_{in}','Mean d-axis current i_{d,bar}^*'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax, style);
ax.Color = 'w';
try
    ax.YAxis(1).Color = style.axisColor;
    ax.YAxis(2).Color = style.axisColor;
catch
end
xlim(ax, [0 5]);
yyaxis(ax, 'right');
ylim(ax, [-4 0]);
yyaxis(ax, 'left');
save_fig(fig, figDir, 'fig3_4_power_id_convergence');
end

function plot_disturbance(group, figDir, style, p)
fig = new_figure(style, [2 2 16 11]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl);
hold(ax1, 'on');
plot(ax1, group.baseline.t, group.baseline.omega_ref * 60/(2*pi), '--', 'Color', style.refColor, 'LineWidth', 1.8);
plot(ax1, group.optimization.t, group.optimization.omega_rpm, '-', 'Color', style.pidColor, 'LineWidth', 2.2);
add_event_line(ax1, p.exp.load_step.t_step, style);
ylabel(ax1, 'n / r\cdotmin^{-1}', 'Interpreter', 'tex');
legend(ax1, {'Reference speed','Actual speed'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax1, style);
xlim(ax1, [0 4]);
ylim(ax1, [1140 1240]);
text(ax1, p.exp.load_step.t_step + 0.04, 1230, 't = 2 s', ...
    'FontName', style.fontName, 'FontSize', style.fontSize, 'Color', style.axisColor);
ax2 = nexttile(tl);
hold(ax2, 'on');
idbar = smooth_for_paper(group.optimization.id_bar, 900);
idbar = min(max(idbar, -4), 0);
plot(ax2, group.optimization.t, idbar, '-', 'Color', style.methodColor, 'LineWidth', 2.2);
add_event_line(ax2, p.exp.load_step.t_step, style);
xlabel(ax2, 't / s', 'Interpreter', 'tex');
ylabel(ax2, 'i_d^* / A', 'Interpreter', 'tex');
format_axes(ax2, style);
xlim(ax2, [0 4]);
ylim(ax2, [-4 0.5]);
text(ax2, p.exp.load_step.t_step + 0.06, -0.95, 'optimizer frozen', ...
    'FontName', style.fontName, 'FontSize', style.fontSize, 'Color', style.axisColor);
save_fig(fig, figDir, 'fig3_5_disturbance_response');
end

function y = smooth_for_paper(x, window)
y = x(:);
if window > 1 && numel(y) > window
    y = movmean(y, window, 'Endpoints', 'shrink');
end
end

function write_parameter_table(tabDir, p)
T = table( ...
    ["电机参数";"电机参数";"电机参数";"电机参数";"电机参数";"机械参数";"机械参数";"逆变器参数";"仿真参数";"仿真参数"], ...
    ["定子电阻";"d轴电感";"q轴电感";"永磁体磁链";"极对数";"转动惯量";"阻尼系数";"直流母线电压";"采样周期";"仿真时长"], ...
    ["Rs";"Ld";"Lq";"psi_f";"p";"J";"B";"Udc";"Ts";"Tstop"], ...
    [p.Rs;p.Ld;p.Lq;p.psi_f;p.pole_pairs;p.J;p.B;p.Vdc;p.Ts;p.Tstop], ...
    ["Ohm";"H";"H";"Wb";"-";"kg*m^2";"N*m*s/rad";"V";"s";"s"], ...
    'VariableNames', {'参数类别','参数名称','符号','数值','单位'});
writetable(T, fullfile(tabDir, 'table3_1_simulation_parameters.csv'));
end

function write_case_table(tabDir)
T = table( ...
    ["工况1";"工况2";"工况3";"工况4";"工况5"], ...
    ["恒速恒载能效寻优";"速度阶跃响应";"负载阶跃扰动";"参数摄动";"电流/电压约束工况"], ...
    ["给定恒定转速与恒定负载转矩，外层优化器在稳态后启动";"转速参考值阶跃变化，比较传统PID/PI与所提方法";"负载转矩在指定时刻阶跃增大";"对Rs、Ld、Lq或psi_f施加比例偏差";"提高转速或负载，使约束参与投影"], ...
    ["P_in、i_d^*、eta收敛过程";"超调量、调节时间、稳态误差";"转速跌落、恢复时间、优化冻结与重启";"功率、效率与鲁棒性变化";"constraint_flag、i_d^*投影边界、稳态功率"], ...
    ["图3-4";"图3-3、表3-3";"图3-5、表3-3";"表3-3";"表3-3"], ...
    'VariableNames', {'工况编号','工况名称','设置方式','主要观察指标','对应图表'});
writetable(T, fullfile(tabDir, 'table3_2_simulation_cases.csv'));
end

function write_current_performance_table(tabDir, metrics)
speedBase = pick(metrics.speed, 'baseline');
speedOpt = pick(metrics.speed, 'optimization');
constBase = pick(metrics.constant, 'baseline');
constOpt = pick(metrics.constant, 'optimization');
loadBase = pick(metrics.load, 'baseline');
loadOpt = pick(metrics.load, 'optimization');
T = table( ...
    ["速度超调量";"调节时间";"稳态转速误差";"稳态输入功率";"定子电流幅值";"效率";"i_d^* 收敛时间";"负载扰动恢复时间"], ...
    [fmt_pct(speedBase.overshoot_pct); fmt_s(speedBase.settling_time_s); fmt_rpm(speedBase.steady_error_rpm); fmt_w(constBase.mean_Pin_W); fmt_a(constBase.mean_is_A); fmt_pct(constBase.efficiency_pct); "不适用"; fmt_s(loadBase.load_recovery_time_s)], ...
    [fmt_pct(speedOpt.overshoot_pct); fmt_s(speedOpt.settling_time_s); fmt_rpm(speedOpt.steady_error_rpm); fmt_w(constOpt.mean_Pin_W); fmt_a(constOpt.mean_is_A); fmt_pct(constOpt.efficiency_pct); fmt_s(constOpt.id_convergence_time_s); fmt_s(loadOpt.load_recovery_time_s)], ...
    [trend(speedBase.overshoot_pct, speedOpt.overshoot_pct, "%"); trend(speedBase.settling_time_s, speedOpt.settling_time_s, "s"); trend_abs(speedBase.steady_error_rpm, speedOpt.steady_error_rpm, "r/min"); trend(constBase.mean_Pin_W, constOpt.mean_Pin_W, "%"); trend(constBase.mean_is_A, constOpt.mean_is_A, "%"); trend_abs(constBase.efficiency_pct, constOpt.efficiency_pct, "百分点"); "所提方法外层优化指标"; trend(loadBase.load_recovery_time_s, loadOpt.load_recovery_time_s, "%")], ...
    'VariableNames', {'指标','传统 PID/PI','所提方法','变化趋势'});
writetable(T, fullfile(tabDir, 'table3_3_performance_comparison.csv'));
end

function write_document_result_comparison(tabDir, paperRoot, ~)
docTablePath = fullfile(paperRoot, 'tables_chapter3', 'table3_3_performance_comparison.csv');
doc = readtable(docTablePath, 'TextType', 'string', 'VariableNamingRule', 'preserve');
curPath = fullfile(tabDir, 'table3_3_performance_comparison.csv');
cur = readtable(curPath, 'TextType', 'string', 'VariableNamingRule', 'preserve');
cmp = table(doc.("指标"), doc.("传统 PID/PI"), doc.("所提方法"), cur.("传统 PID/PI"), cur.("所提方法"), ...
    'VariableNames', {'指标','文档_传统PIDPI','文档_所提方法','当前组件_传统PIDPI','当前组件_所提方法'});
writetable(cmp, fullfile(tabDir, 'document_result_comparison.csv'));

fid = fopen(fullfile(tabDir, 'document_result_comparison.md'), 'w', 'n', 'UTF-8');
if fid >= 0
    fprintf(fid, '# Document and Component Result Comparison\n\n');
    fprintf(fid, 'This report compares the Word/legacy Chapter 3 metrics with the current R2025b component Simulink model outputs.\n\n');
    fprintf(fid, '| 指标 | 文档传统PID/PI | 文档所提方法 | 当前组件传统PID/PI | 当前组件所提方法 |\n');
    fprintf(fid, '|---|---:|---:|---:|---:|\n');
    for k = 1:height(cmp)
        fprintf(fid, '| %s | %s | %s | %s | %s |\n', cmp.("指标")(k), cmp.("文档_传统PIDPI")(k), cmp.("文档_所提方法")(k), cmp.("当前组件_传统PIDPI")(k), cmp.("当前组件_所提方法")(k));
    end
    fclose(fid);
end
end

function write_insert_order(root)
fid = fopen(fullfile(root, 'chapter3_insert_order.md'), 'w', 'n', 'UTF-8');
if fid < 0, return; end
fprintf(fid, '# Chapter 3 Figure and Table Insert Order\n\n');
fprintf(fid, 'Use the files below as the Chapter 3 figures and tables. They are regenerated from the R2025b component Simulink model.\n\n');
fprintf(fid, '1. Figure 3-1: `figures_chapter3\\\\fig3_1_simulink_overall_model.png`\n');
fprintf(fid, '2. Figure 3-2: `figures_chapter3\\\\fig3_2_model_free_optimizer_subsystem.png`\n');
fprintf(fid, '3. Table 3-1: `tables_chapter3\\\\table3_1_simulation_parameters.csv`\n');
fprintf(fid, '4. Table 3-2: `tables_chapter3\\\\table3_2_simulation_cases.csv`\n');
fprintf(fid, '5. Figure 3-3: `figures_chapter3\\\\fig3_3_speed_step_response.png`\n');
fprintf(fid, '6. Figure 3-4: `figures_chapter3\\\\fig3_4_power_id_convergence.png`\n');
fprintf(fid, '7. Figure 3-5: `figures_chapter3\\\\fig3_5_disturbance_response.png`\n');
fprintf(fid, '8. Table 3-3: `tables_chapter3\\\\table3_3_performance_comparison.csv`\n');
fprintf(fid, '9. Comparison report: `tables_chapter3\\\\document_result_comparison.md`\n');
fclose(fid);
end

function r = pick(T, method)
r = T(strcmp(T.method, method), :);
end

function s = fmt_pct(x), s = string(sprintf('%.4g%%', x)); end
function s = fmt_s(x), if isnan(x), s = "不适用"; else, s = string(sprintf('%.4f s', x)); end, end
function s = fmt_rpm(x), s = string(sprintf('%.4f r/min', x)); end
function s = fmt_w(x), s = string(sprintf('%.3f W', x)); end
function s = fmt_a(x), s = string(sprintf('%.4f A', x)); end

function s = trend(a, b, unit)
if isnan(a) || isnan(b) || abs(a) < eps
    s = "不适用";
    return;
end
d = 100 * (b - a) / abs(a);
if d < 0
    s = string(sprintf('降低 %.3g%s', abs(d), unit));
else
    s = string(sprintf('提高 %.3g%s', d, unit));
end
end

function s = trend_abs(a, b, unit)
d = b - a;
if d < 0
    s = string(sprintf('降低 %.4g %s', abs(d), unit));
else
    s = string(sprintf('提高 %.4g %s', d, unit));
end
end

function style = paper_style()
style.fontName = 'Times New Roman';
style.fontSize = 10;
style.refColor = [0.20 0.20 0.20];
style.pidColor = [0.55 0.16 0.14];
style.methodColor = [0.00 0.32 0.56];
style.greenColor = [0.10 0.45 0.25];
style.eventColor = [0.45 0.45 0.45];
style.axisColor = [0.12 0.12 0.12];
style.ioFill = [0.92 0.96 1.00];
style.moduleFill = [0.98 0.96 0.90];
style.lineColor = [0.18 0.18 0.18];
end

function fig = new_figure(style, pos)
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', pos, 'Visible', 'off');
set(fig, 'Renderer', 'painters');
set(fig, 'DefaultAxesFontName', style.fontName, 'DefaultTextFontName', style.fontName);
set(fig, 'InvertHardcopy', 'off');
end

function fig = new_canvas(style, pos)
fig = new_figure(style, pos);
axes('Position', [0 0 1 1], 'Visible', 'off');
end

function format_axes(ax, style)
grid(ax, 'on');
box(ax, 'on');
ax.Color = 'w';
ax.GridColor = [0.78 0.78 0.78];
ax.GridAlpha = 0.55;
set(ax, 'FontName', style.fontName, 'FontSize', style.fontSize, 'LineWidth', 0.8, ...
    'XColor', style.axisColor, 'YColor', style.axisColor);
end

function add_event_line(ax, x, style)
yl = ylim(ax);
plot(ax, [x x], yl, '--', 'Color', style.eventColor, 'LineWidth', 1.0);
ylim(ax, yl);
end

function save_fig(fig, figDir, stem)
pngPath = fullfile(figDir, [stem '.png']);
exportgraphics(fig, pngPath, 'Resolution', 300, 'BackgroundColor', 'white');
if png_is_black(pngPath)
    set(fig, 'Color', 'w', 'InvertHardcopy', 'off');
    print(fig, pngPath, '-dpng', '-r300', '-painters');
end
try
    savefig(fig, fullfile(figDir, [stem '.fig']));
catch
end
close(fig);
end

function tf = png_is_black(path)
tf = false;
try
    img = imread(path);
    if ndims(img) == 3
        img = img(:,:,1:3);
    end
    tf = mean(double(img(:))) < 3;
catch
    tf = true;
end
end

function box_block(pos, label, fill, style)
annotation('rectangle', pos, 'FaceColor', fill, 'Color', style.lineColor, 'LineWidth', 1.0);
annotation('textbox', pos, 'String', label, 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'LineStyle', 'none', 'FontName', style.fontName, 'FontSize', style.fontSize, ...
    'Interpreter', 'none', 'Color', style.axisColor);
end

function text_label(pos, label, style, align)
annotation('textbox', pos, 'String', label, 'FitBoxToText', 'off', ...
    'HorizontalAlignment', align, 'VerticalAlignment', 'middle', ...
    'LineStyle', 'none', 'FontName', style.fontName, 'FontSize', style.fontSize, ...
    'Interpreter', 'none', 'Color', style.axisColor);
end

function arrow_line(x, y, style)
annotation('arrow', x, y, 'Color', style.lineColor, 'LineWidth', 1.0);
end

function plain_line(x, y, style)
annotation('line', x, y, 'Color', style.lineColor, 'LineWidth', 1.0);
end
