function varargout = paper_component_tools(action, varargin)
%PAPER_COMPONENT_TOOLS Split runner for real R2025b component paper figures.

switch lower(string(action))
    case "prepare"
        p = paper_tuned_parameters();
        assignin('base', 'p', p);
        run(fullfile(p.paths.scripts, 'build_component_models.m'));
        varargout = {p};
    case "run_case"
        p = paper_tuned_parameters();
        caseName = string(varargin{1});
        group = run_pair(make_scenario(caseName, p), p);
        save(fullfile(p.paths.data, "paper_case_" + caseName + ".mat"), 'group', 'p');
        varargout = {group};
    case "merge_plot"
        p = paper_tuned_parameters();
        data = load_cases(p);
        metrics = struct();
        metrics.constant = metrics_pair(data.constant, p, 'constant_load');
        metrics.speed = metrics_pair(data.speed, p, 'speed_step');
        metrics.load = metrics_pair(data.load, p, 'load_step');
        save(fullfile(p.paths.data, 'paper_aligned_component_results.mat'), ...
            '-struct', 'data', 'constant', 'speedStep', 'loadStep');
        save(fullfile(p.paths.data, 'paper_aligned_component_results.mat'), ...
            'metrics', 'p', '-append');
        plot_figures(data, p);
        write_tables(metrics, p);
        varargout = {metrics};
    otherwise
        error('Unknown action: %s', action);
end
end

function p = paper_tuned_parameters()
run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
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
p.optimizer_for_component.search_bias_rate = 1.15;
p.optimizer_for_component.search_bias_tau = 0.40;
p.optimizer_for_component.search_bias_stop_id = -3.85;
p.steady.hold_time = 0.42;
p.steady.power_slope_threshold = 1.0e6;
p.paper_figures.constant_omega_rpm = 1200;
p.paper_figures.constant_TL = 5.0;
p.paper_figures.load_TL1 = 2.0;
p.paper_figures.load_TL2 = 5.0;
end

function scenario = make_scenario(caseName, p)
switch lower(caseName)
    case "constant"
        scenario.name = 'constant_load';
        scenario.Tstop = 5.0;
        scenario.initial_omega_rpm = p.paper_figures.constant_omega_rpm;
        scenario.initial_load_torque = p.paper_figures.constant_TL;
        scenario.omega_ref = @(t) p.paper_figures.constant_omega_rpm * 2*pi/60;
        scenario.load_torque = @(t) p.paper_figures.constant_TL;
    case "speed"
        scenario.name = 'speed_step';
        scenario.Tstop = 5.0;
        scenario.initial_omega_rpm = p.exp.speed_step.omega1_rpm;
        scenario.initial_load_torque = p.exp.speed_step.TL;
        scenario.omega_ref = @(t) (t < p.exp.speed_step.t_step) * (p.exp.speed_step.omega1_rpm * 2*pi/60) + ...
            (t >= p.exp.speed_step.t_step) * (p.exp.speed_step.omega2_rpm * 2*pi/60);
        scenario.load_torque = @(t) p.exp.speed_step.TL;
    case "load"
        scenario.name = 'load_step';
        scenario.Tstop = 5.0;
        scenario.initial_omega_rpm = p.exp.load_step.omega_rpm;
        scenario.initial_load_torque = p.paper_figures.load_TL1;
        scenario.omega_ref = @(t) p.exp.load_step.omega_rpm * 2*pi/60;
        scenario.load_torque = @(t) p.paper_figures.load_TL1 + ...
            (t >= p.exp.load_step.t_step) * (p.paper_figures.load_TL2 - p.paper_figures.load_TL1);
    otherwise
        error('Unknown case: %s', caseName);
end
end

function group = run_pair(scenario, p)
group.baseline = run_component_model('PMSM_FOC_Component_Baseline', scenario, p);
group.optimization = run_component_model('PMSM_FOC_Component_Optimization', scenario, p);
group.scenario = scenario;
end

function run = run_component_model(modelName, scenario, p)
load_system(fullfile(p.paths.models, [modelName '.slx']));
t = (0:p.Ts:scenario.Tstop).';
omegaRef = arrayfun(scenario.omega_ref, t);
loadTorque = arrayfun(scenario.load_torque, t);
ds = Simulink.SimulationData.Dataset;
ds{1} = timeseries(omegaRef, t);
ds{2} = timeseries(loadTorque, t);
in = Simulink.SimulationInput(modelName);
in = in.setModelParameter('StopTime', num2str(scenario.Tstop), ...
    'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
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
omega = scenario.initial_omega_rpm * 2*pi/60;
TL = scenario.initial_load_torque;
id0 = 0;
iq0 = (TL + p.B * omega) / (1.5 * p.pole_pairs * p.psi_f);
omega_e = p.pole_pairs * omega;
ud0 = -omega_e * p.Lq * iq0;
uq0 = p.Rs * iq0 + omega_e * (p.Ld * id0 + p.psi_f);
Pin0 = 1.5 * (ud0 * id0 + uq0 * iq0);
us0 = hypot(ud0, uq0);
speedInt0 = iq0 / effective_speed_ki(modelName, p);
iqInt0 = (p.Rs * iq0) / p.current.q.Ki;
ics = {'z_id', id0; 'z_iq', iq0; 'z_omega_m', omega; ...
    'z_speed_int', speedInt0; 'z_id_int', 0; 'z_iq_int', iqInt0; ...
    'z_Pin', Pin0; 'z_Pin_prev', Pin0; 'z_Pin_lpf', Pin0; ...
    'z_g_hat_lpf', 0; 'z_id_bar', 0; 'z_u_s', us0; ...
    'z_id_ref_prev', 0; 'z_steady_count', 0; 'z_perturb_ramp', 0; ...
    'z_current_sat', 0; 'z_voltage_sat', 0};
for k = 1:size(ics, 1)
    in = in.setBlockParameter([modelName '/' ics{k, 1}], ...
        'InitialCondition', num2str(ics{k, 2}, 17));
end

function Ki = effective_speed_ki(modelName, p)
Ki = p.speed.Ki;
if contains(modelName, 'Baseline') && isfield(p, 'speed_by_method') && isfield(p.speed_by_method, 'baseline')
    Ki = p.speed_by_method.baseline.Ki;
elseif contains(modelName, 'Optimization') && isfield(p, 'speed_by_method') && isfield(p.speed_by_method, 'optimization')
    Ki = p.speed_by_method.optimization.Ki;
end
end
end

function data = load_cases(p)
S = load(fullfile(p.paths.data, 'paper_case_constant.mat')); data.constant = S.group;
S = load(fullfile(p.paths.data, 'paper_case_speed.mat')); data.speedStep = S.group; data.speed = S.group;
S = load(fullfile(p.paths.data, 'paper_case_load.mat')); data.loadStep = S.group; data.load = S.group;
end

function plot_figures(data, p)
figDir = fullfile(p.project_root, 'figures_chapter3');
if exist(figDir, 'dir') ~= 7, mkdir(figDir); end
style = paper_style();
plot_speed(data.speed, figDir, style);
plot_power_id(data.constant, figDir, style);
plot_disturbance(data.load, figDir, style, p);
end

function plot_speed(group, figDir, style)
fig = new_figure(style, [2 2 16 9]); ax = axes(fig); hold(ax, 'on');
plot(ax, group.baseline.t, group.baseline.omega_ref * 60/(2*pi), '--', 'Color', style.refColor, 'LineWidth', 1.8);
plot(ax, group.baseline.t, group.baseline.omega_rpm, '-', 'Color', style.pidColor, 'LineWidth', 2.2);
plot(ax, group.optimization.t, group.optimization.omega_rpm, '-', 'Color', style.methodColor, 'LineWidth', 2.2);
xlabel(ax, 't / s'); ylabel(ax, 'n / r\cdotmin^{-1}', 'Interpreter', 'tex');
lgd = legend(ax, {'Reference speed','Conventional PID/PI','Proposed method'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_legend(lgd, style);
format_axes(ax, style);
xlim(ax, [0.46 0.92]);
ylim(ax, [960 1800]);
save_fig(fig, figDir, 'fig3_3_speed_step_response');
end

function plot_power_id(group, figDir, style)
fig = new_figure(style, [2 2 16 9]); ax = axes(fig);
t = group.optimization.t;
pin = smooth_for_paper(group.optimization.Pin, 900);
idbar = smooth_for_paper(group.optimization.id_bar, 700);
yyaxis(ax, 'left');
plot(ax, t, pin, '-', 'Color', style.pidColor, 'LineWidth', 2.2);
ylabel(ax, 'P_{in} / W', 'Interpreter', 'tex');
yyaxis(ax, 'right');
plot(ax, t, idbar, '-', 'Color', style.methodColor, 'LineWidth', 2.2);
ylabel(ax, 'i_{d,bar}^* / A', 'Interpreter', 'tex');
xlabel(ax, 't / s');
lgd = legend(ax, {'Input power P_{in}','Mean d-axis current i_{d,bar}^*'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_legend(lgd, style);
format_axes(ax, style); ax.YAxis(1).Color = style.axisColor; ax.YAxis(2).Color = style.axisColor;
xlim(ax, [0 5]);
yyaxis(ax, 'right');
ylim(ax, [-4 0]);
yyaxis(ax, 'left');
ylim(ax, nice_limits(pin, 2.0));
save_fig(fig, figDir, 'fig3_4_power_id_convergence');
end

function plot_disturbance(group, figDir, style, p)
fig = new_figure(style, [2 2 16 11]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on');
plot(ax1, group.optimization.t, group.optimization.omega_ref * 60/(2*pi), '--', 'Color', style.refColor, 'LineWidth', 1.8);
plot(ax1, group.optimization.t, group.optimization.omega_rpm, '-', 'Color', style.methodColor, 'LineWidth', 2.2);
add_event_line(ax1, p.exp.load_step.t_step, style);
ylabel(ax1, 'n / r\cdotmin^{-1}', 'Interpreter', 'tex');
lgd = legend(ax1, {'Reference speed','Actual speed'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_legend(lgd, style);
format_axes(ax1, style);
xlim(ax1, [1.94 2.22]);
ylim(ax1, [1105 1208]);
text(ax1, p.exp.load_step.t_step + 0.012, 1204, 't = 2 s', 'FontName', style.fontName, 'FontSize', style.fontSize, 'Color', style.axisColor);
ax2 = nexttile(tl); hold(ax2, 'on');
idbar = smooth_for_paper(group.optimization.id_bar, 700);
plot(ax2, group.optimization.t, idbar, '-', 'Color', style.pidColor, 'LineWidth', 2.2);
add_event_line(ax2, p.exp.load_step.t_step, style);
xlabel(ax2, 't / s'); ylabel(ax2, 'i_d^* / A', 'Interpreter', 'tex');
format_axes(ax2, style);
xlim(ax2, [0.35 4.25]);
ylim(ax2, [-3.6 0.2]);
text(ax2, p.exp.load_step.t_step + 0.08, -2.35, 'optimizer frozen', 'FontName', style.fontName, 'FontSize', style.fontSize, 'Color', style.axisColor);
save_fig(fig, figDir, 'fig3_5_disturbance_response');
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

function write_tables(metrics, p)
tabDir = fullfile(p.project_root, 'tables_chapter3');
if exist(tabDir, 'dir') ~= 7, mkdir(tabDir); end
speedBase = pick(metrics.speed, 'baseline'); speedOpt = pick(metrics.speed, 'optimization');
constBase = pick(metrics.constant, 'baseline'); constOpt = pick(metrics.constant, 'optimization');
loadBase = pick(metrics.load, 'baseline'); loadOpt = pick(metrics.load, 'optimization');
T = table(["速度超调量";"调节时间";"稳态转速误差";"稳态输入功率";"定子电流幅值";"效率";"i_d^* 收敛时间";"负载扰动恢复时间"], ...
    [fmt_pct(speedBase.overshoot_pct); fmt_s(speedBase.settling_time_s); fmt_rpm(speedBase.steady_error_rpm); fmt_w(constBase.mean_Pin_W); fmt_a(constBase.mean_is_A); fmt_pct(constBase.efficiency_pct); "不适用"; fmt_s(loadBase.load_recovery_time_s)], ...
    [fmt_pct(speedOpt.overshoot_pct); fmt_s(speedOpt.settling_time_s); fmt_rpm(speedOpt.steady_error_rpm); fmt_w(constOpt.mean_Pin_W); fmt_a(constOpt.mean_is_A); fmt_pct(constOpt.efficiency_pct); fmt_s(constOpt.id_convergence_time_s); fmt_s(loadOpt.load_recovery_time_s)], ...
    'VariableNames', {'指标','传统 PID/PI','所提方法'});
writetable(T, fullfile(tabDir, 'table3_3_performance_comparison.csv'));
end

function tc = id_convergence(run, method, p)
if ~strcmp(method, 'optimization'), tc = NaN; return; end
tail = run.t >= run.t(end) * (1 - p.metric_tail_fraction);
target = mean(run.id_bar(tail));
inside = abs(run.id_bar - target) <= max(0.08, 0.20 * abs(target));
idx = find(inside & run.optimizer_enable > 0, 1, 'first');
if isempty(idx), tc = NaN; else, tc = run.t(idx); end
end

function ts = settling_time(t, y, target, tol)
inside = abs(y - target) <= max(abs(target) * tol, 1e-6);
suffix = flipud(cumprod(flipud(double(inside))) > 0);
idx = find(suffix, 1, 'first');
if isempty(idx), ts = NaN; else, ts = t(idx); end
end

function tr = load_recovery(run)
idx = find(abs(diff(run.T_L)) > 1e-12, 1, 'first');
if isempty(idx), tr = NaN; return; end
idx = idx + 1;
target = run.omega_ref(end) * 60/(2*pi);
band = 0.02 * max(abs(target), 1);
leftBand = find(abs(run.omega_rpm(idx:end) - target) > band, 1, 'first');
if isempty(leftBand), tr = 0; return; end
searchStart = idx + leftBand - 1;
post = find(abs(run.omega_rpm(searchStart:end) - target) <= band, 1, 'first');
if isempty(post), tr = NaN; else, tr = run.t(searchStart + post - 1) - run.t(idx); end
end

function y = smooth_for_paper(x, window)
y = x(:);
if window > 1 && numel(y) > window
    y = movmean(y, window, 'Endpoints', 'shrink');
end
end

function style = paper_style()
style.fontName = 'Times New Roman';
style.fontSize = 10;
style.refColor = [0.20 0.20 0.20];
style.pidColor = [0.55 0.16 0.14];
style.methodColor = [0.00 0.32 0.56];
style.eventColor = [0.45 0.45 0.45];
style.axisColor = [0.12 0.12 0.12];
end

function fig = new_figure(style, pos)
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', pos, 'Visible', 'off');
set(fig, 'DefaultAxesFontName', style.fontName, 'DefaultTextFontName', style.fontName, 'InvertHardcopy', 'off');
end

function format_axes(ax, style)
grid(ax, 'on'); box(ax, 'on'); ax.Color = 'w'; ax.GridColor = [0.78 0.78 0.78]; ax.GridAlpha = 0.55;
set(ax, 'FontName', style.fontName, 'FontSize', style.fontSize, 'LineWidth', 0.8, 'XColor', style.axisColor, 'YColor', style.axisColor);
end

function style_legend(lgd, style)
set(lgd, 'TextColor', style.axisColor, 'FontName', style.fontName, ...
    'FontSize', style.fontSize, 'Color', 'none');
end

function lim = nice_limits(y, pad)
y = y(isfinite(y));
lo = min(y);
hi = max(y);
if hi <= lo
    hi = lo + 1;
end
lo = floor((lo - pad) / 5) * 5;
hi = ceil((hi + pad) / 5) * 5;
lim = [lo hi];
end

function add_event_line(ax, x, style)
yl = ylim(ax); plot(ax, [x x], yl, '--', 'Color', style.eventColor, 'LineWidth', 1.0); ylim(ax, yl);
end

function save_fig(fig, figDir, stem)
pngPath = fullfile(figDir, [stem '.png']);
if ~isgraphics(fig, 'figure')
    error('Figure handle for %s is invalid before export.', stem);
end
set(fig, 'Color', 'w', 'InvertHardcopy', 'off');
drawnow;
print(fig, pngPath, '-dpng', '-r300', '-image');
drawnow;
if png_is_black(pngPath)
    exportgraphics(fig, pngPath, 'Resolution', 300, 'BackgroundColor', 'white');
end
if png_is_black(pngPath)
    set(fig, 'Visible', 'on');
    drawnow;
    print(fig, pngPath, '-dpng', '-r300', '-image');
end
try
    savefig(fig, fullfile(figDir, [stem '.fig']));
catch
end
close(fig);
end

function tf = png_is_black(path)
try
    img = imread(path);
    tf = mean(double(img(:))) < 3;
catch
    tf = true;
end
end

function r = pick(T, method), r = T(strcmp(T.method, method), :); end
function s = fmt_pct(x), s = string(sprintf('%.4g%%', x)); end
function s = fmt_s(x), if isnan(x), s = "不适用"; else, s = string(sprintf('%.4f s', x)); end, end
function s = fmt_rpm(x), s = string(sprintf('%.4f r/min', x)); end
function s = fmt_w(x), s = string(sprintf('%.3f W', x)); end
function s = fmt_a(x), s = string(sprintf('%.4f A', x)); end
