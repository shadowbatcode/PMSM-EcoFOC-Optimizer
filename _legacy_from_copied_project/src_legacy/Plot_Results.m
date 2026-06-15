function Plot_Results(allResults, metrics, p)
%PLOT_RESULTS Generate paper-ready PNG and FIG outputs from simulation data.

style = plotStyle();

plotModelStructure(p, style);
plotConstantSpeed(allResults.constant_load, p, style);
plotConstantId(allResults.constant_load, p, style);
plotConstantPower(allResults.constant_load, p, style);
plotLoadStepSpeedCurrent(allResults.load_step, p, style);
plotLoadStepFreeze(allResults.load_step, p, style);
plotSpeedStepResponse(allResults.speed_step, p, style);
plotSpeedStepFlag(allResults.speed_step, p, style);
plotParameterEfficiency(allResults.parameter_perturbation, p, style);
plotNoiseDelayGradient(allResults.noise_delay, p, style);
plotConstraintProjection(allResults.constraints, p, style);

if nargin > 1 && ~isempty(metrics)
    % Metrics are generated before plotting and intentionally kept available
    % for future figure annotations without changing the plotting API.
end
end

function plotModelStructure(p, style)
fig = newFigure(style, [100 100 1100 620]);
axis off;

flowBox(fig, 0.04, 0.64, 0.15, 0.13, 'Speed reference', style.blue);
flowBox(fig, 0.25, 0.64, 0.16, 0.13, 'Speed PI/PID', style.green);
flowBox(fig, 0.47, 0.64, 0.18, 0.13, 'd/q current PI', style.orange);
flowBox(fig, 0.71, 0.64, 0.15, 0.13, 'PMSM dq plant', style.yellow);
flowBox(fig, 0.71, 0.34, 0.15, 0.13, 'Power monitor', style.blue);
flowBox(fig, 0.47, 0.34, 0.18, 0.13, 'Steady detector', style.green);
flowBox(fig, 0.25, 0.34, 0.16, 0.13, 'MTPA / MFO', style.green);

flowArrow(fig, [0.19 0.25], [0.705 0.705], style.dark);
flowArrow(fig, [0.41 0.47], [0.705 0.705], style.dark);
flowArrow(fig, [0.65 0.71], [0.705 0.705], style.dark);
flowArrow(fig, [0.79 0.79], [0.64 0.47], style.dark);
flowArrow(fig, [0.71 0.65], [0.405 0.405], style.dark);
flowArrow(fig, [0.47 0.41], [0.405 0.405], style.dark);
flowArrow(fig, [0.33 0.33], [0.47 0.64], style.dark);
flowArrow(fig, [0.71 0.56], [0.64 0.47], style.gray);

annotation(fig, 'textbox', [0.08 0.12 0.84 0.12], ...
    'String', 'Generated architecture: speed-loop regulation, current-loop vector control, dq PMSM plant, and power-feedback model-free d-axis current optimization.', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'LineStyle', 'none', ...
    'FontName', style.fontName, ...
    'FontSize', style.fontSize + 1, ...
    'Color', style.dark, ...
    'Interpreter', 'none');

title('PMSM FOC model-free optimization structure');
saveFigure(fig, p, 'fig_3_1_model_structure');
end

function plotConstantSpeed(group, p, style)
fig = newFigure(style);
ax = axes(fig);
hold(ax, 'on');
ref = plotSpeedReference(ax, group.mfo, style);
lines = plotMethods(ax, group, 'omega_rpm', p, style);
xlabel(ax, 'Time / s');
ylabel(ax, 'Speed / r/min');
title(ax, 'Constant-load speed response');
legend(ax, [ref; lines], [{'Reference'}, methodLabels(p)], 'Location', 'best');
formatLegend(ax, style);
formatAxes(ax, style);
setTimeLimits(ax, group.mfo.t);
saveFigure(fig, p, 'fig_3_2_constant_speed_response');
end

function plotConstantId(group, p, style)
fig = newFigure(style);
ax = axes(fig);
hold(ax, 'on');
plotMethods(ax, group, 'id_ref', p, style);
xlabel(ax, 'Time / s');
ylabel(ax, 'd-axis current reference / A');
title(ax, 'd-axis current reference convergence');
legend(ax, methodLabels(p), 'Location', 'best');
formatLegend(ax, style);
formatAxes(ax, style);
setTimeLimits(ax, group.mfo.t);
saveFigure(fig, p, 'fig_3_3_constant_id_convergence');
end

function plotConstantPower(group, p, style)
fig = newFigure(style, [100 100 980 680]);
tl = newTiles(fig, 2, 1);

ax1 = nexttile(tl);
hold(ax1, 'on');
plotMethods(ax1, group, 'P_in', p, style);
ylabel(ax1, 'Input power / W');
title(ax1, 'Instantaneous input power');
legend(ax1, methodLabels(p), 'Location', 'best');
formatLegend(ax1, style);
formatAxes(ax1, style);
setTimeLimits(ax1, group.mfo.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
plotMethods(ax2, group, 'Pbar_in', p, style);
xlabel(ax2, 'Time / s');
ylabel(ax2, 'Filtered input power / W');
title(ax2, 'Filtered average input power');
formatAxes(ax2, style);
setTimeLimits(ax2, group.mfo.t);

saveFigure(fig, p, 'fig_3_4_constant_power_comparison');
end

function plotLoadStepSpeedCurrent(group, p, style)
fig = newFigure(style, [100 100 980 760]);
tl = newTiles(fig, 3, 1);

ax1 = nexttile(tl);
hold(ax1, 'on');
ref = plotSpeedReference(ax1, group.mfo, style);
lines = plotMethods(ax1, group, 'omega_rpm', p, style);
ylabel(ax1, 'Speed / r/min');
title(ax1, 'Load-step speed response');
legend(ax1, [ref; lines], [{'Reference'}, methodLabels(p)], 'Location', 'best');
formatLegend(ax1, style);
formatAxes(ax1, style);
setTimeLimits(ax1, group.mfo.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
plotMethods(ax2, group, 'iq', p, style);
ylabel(ax2, 'q-axis current / A');
title(ax2, 'q-axis current response');
formatAxes(ax2, style);
setTimeLimits(ax2, group.mfo.t);

ax3 = nexttile(tl);
plot(ax3, group.mfo.t, group.mfo.T_L, 'Color', style.dark, 'LineWidth', style.lineWidth);
xlabel(ax3, 'Time / s');
ylabel(ax3, 'Load torque / N*m');
title(ax3, 'Load torque');
formatAxes(ax3, style);
setTimeLimits(ax3, group.mfo.t);

saveFigure(fig, p, 'fig_3_5_load_step_speed_current');
end

function plotLoadStepFreeze(group, p, style)
run = group.mfo;
fig = newFigure(style, [100 100 980 760]);
tl = newTiles(fig, 3, 1);

ax1 = nexttile(tl);
plot(ax1, run.t, run.id_ref, 'Color', style.colors(3,:), 'LineWidth', style.lineWidth);
ylabel(ax1, 'i_d^* / A');
title(ax1, 'MFO freeze and restart: d-axis reference');
formatAxes(ax1, style);
setTimeLimits(ax1, run.t);

ax2 = nexttile(tl);
stairs(ax2, run.t, run.steady_flag, 'Color', style.colors(1,:), 'LineWidth', style.lineWidth);
ylabel(ax2, 'S(k)');
title(ax2, 'Steady-state detector flag');
formatFlagAxes(ax2, style);
setTimeLimits(ax2, run.t);

ax3 = nexttile(tl);
stairs(ax3, run.t, run.optimizer_active, 'Color', style.colors(2,:), 'LineWidth', style.lineWidth);
xlabel(ax3, 'Time / s');
ylabel(ax3, 'Optimizer active');
title(ax3, 'Optimizer enable signal');
formatFlagAxes(ax3, style);
setTimeLimits(ax3, run.t);

saveFigure(fig, p, 'fig_3_6_load_step_freeze_restart');
end

function plotSpeedStepResponse(group, p, style)
fig = newFigure(style, [100 100 980 760]);
tl = newTiles(fig, 3, 1);

ax1 = nexttile(tl);
hold(ax1, 'on');
ref = plotSpeedReference(ax1, group.mfo, style);
lines = plotMethods(ax1, group, 'omega_rpm', p, style);
ylabel(ax1, 'Speed / r/min');
title(ax1, 'Speed-step response');
legend(ax1, [ref; lines], [{'Reference'}, methodLabels(p)], 'Location', 'best');
formatLegend(ax1, style);
formatAxes(ax1, style);
setTimeLimits(ax1, group.mfo.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
plotMethods(ax2, group, 'id_ref', p, style);
ylabel(ax2, 'i_d^* / A');
title(ax2, 'd-axis current reference');
formatAxes(ax2, style);
setTimeLimits(ax2, group.mfo.t);

ax3 = nexttile(tl);
hold(ax3, 'on');
plotMethods(ax3, group, 'iq', p, style);
xlabel(ax3, 'Time / s');
ylabel(ax3, 'i_q / A');
title(ax3, 'q-axis current');
formatAxes(ax3, style);
setTimeLimits(ax3, group.mfo.t);

saveFigure(fig, p, 'fig_3_7_speed_step_response');
end

function plotSpeedStepFlag(group, p, style)
run = group.mfo;
fig = newFigure(style, [100 100 980 680]);
tl = newTiles(fig, 2, 1);

ax1 = nexttile(tl);
stairs(ax1, run.t, run.steady_flag, 'Color', style.colors(1,:), 'LineWidth', style.lineWidth);
ylabel(ax1, 'S(k)');
title(ax1, 'Dynamic and steady-state switching flag');
formatFlagAxes(ax1, style);
setTimeLimits(ax1, run.t);

ax2 = nexttile(tl);
plot(ax2, run.t, run.P_in, 'Color', style.colors(3,:), 'LineWidth', style.lineWidth);
xlabel(ax2, 'Time / s');
ylabel(ax2, 'Input power / W');
title(ax2, 'MFO input power during speed step');
formatAxes(ax2, style);
setTimeLimits(ax2, run.t);

saveFigure(fig, p, 'fig_3_8_speed_step_switching_flag');
end

function plotParameterEfficiency(result, p, style)
methods = p.method_order;
vals = zeros(numel(methods), 2);
for i = 1:numel(methods)
    vals(i, 1) = steadyMean(result.nominal.(methods{i}).eta, p) * 100;
    vals(i, 2) = steadyMean(result.perturbed.(methods{i}).eta, p) * 100;
end

fig = newFigure(style, [100 100 920 620]);
ax = axes(fig);
b = bar(ax, vals, 'grouped', 'LineWidth', 0.8);
b(1).FaceColor = style.colors(1,:);
b(2).FaceColor = style.colors(3,:);
set(ax, 'XTick', 1:numel(methods), 'XTickLabel', methodLabels(p));
ylabel(ax, 'Mean efficiency / %');
title(ax, 'Parameter perturbation efficiency comparison');
legend(ax, {'Nominal plant', 'Perturbed plant'}, 'Location', 'best');
formatLegend(ax, style);
formatAxes(ax, style);
ylim(ax, [max(0, min(vals(:)) - 4), min(100, max(vals(:)) + 4)]);

try
    for j = 1:numel(b)
        text(ax, b(j).XEndPoints, b(j).YEndPoints + 0.25, ...
            compose('%.1f', b(j).YData), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontName', style.fontName, ...
            'FontSize', style.fontSize - 1, ...
            'Color', style.dark);
    end
catch
end

saveFigure(fig, p, 'fig_3_9_parameter_perturbation_efficiency');
end

function plotNoiseDelayGradient(result, p, style)
clean = result.clean.mfo;
nonideal = result.nonideal.mfo;
fig = newFigure(style, [100 100 980 760]);
tl = newTiles(fig, 3, 1);

ax1 = nexttile(tl);
plotComparison(ax1, clean.t, clean.g_hat, nonideal.t, nonideal.g_hat, style);
ylabel(ax1, 'g hat / W/A');
title(ax1, 'Gradient estimate under nonideal factors');
legend(ax1, {'Clean', 'Noise and delay'}, 'Location', 'best');
formatLegend(ax1, style);
formatAxes(ax1, style);
setTimeLimits(ax1, clean.t);

ax2 = nexttile(tl);
plotComparison(ax2, clean.t, clean.id_ref, nonideal.t, nonideal.id_ref, style);
ylabel(ax2, 'i_d^* / A');
title(ax2, 'd-axis current reference');
formatAxes(ax2, style);
setTimeLimits(ax2, clean.t);

ax3 = nexttile(tl);
plotComparison(ax3, clean.t, clean.Pbar_in, nonideal.t, nonideal.Pbar_in, style);
xlabel(ax3, 'Time / s');
ylabel(ax3, 'Filtered input power / W');
title(ax3, 'Filtered input power');
formatAxes(ax3, style);
setTimeLimits(ax3, clean.t);

saveFigure(fig, p, 'fig_3_10_noise_delay_gradient');
end

function plotConstraintProjection(group, p, style)
run = group.mfo;
fig = newFigure(style, [100 100 980 760]);
tl = newTiles(fig, 3, 1);

ax1 = nexttile(tl);
hold(ax1, 'on');
plot(ax1, run.t, run.id_bar_candidate, 'Color', style.colors(1,:), 'LineWidth', style.lineWidth);
plot(ax1, run.t, run.id_bar_projected, 'Color', style.colors(3,:), 'LineWidth', style.lineWidth);
ylabel(ax1, 'bar i_d^* / A');
title(ax1, 'Projection before and after constraints');
legend(ax1, {'Before projection', 'After projection'}, 'Location', 'best');
formatLegend(ax1, style);
formatAxes(ax1, style);
setTimeLimits(ax1, run.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
plot(ax2, run.t, run.i_s, 'Color', style.colors(2,:), 'LineWidth', style.lineWidth);
plot(ax2, run.t, p.i_s_max * ones(size(run.t)), '--', 'Color', style.dark, 'LineWidth', style.refLineWidth);
ylabel(ax2, 'i_s / A');
title(ax2, 'Current magnitude limit');
legend(ax2, {'i_s', 'i_s,max'}, 'Location', 'best');
formatLegend(ax2, style);
formatAxes(ax2, style);
setTimeLimits(ax2, run.t);

ax3 = nexttile(tl);
hold(ax3, 'on');
plot(ax3, run.t, run.u_s, 'Color', style.colors(1,:), 'LineWidth', style.lineWidth);
plot(ax3, run.t, run.U_dc ./ sqrt(3), '--', 'Color', style.dark, 'LineWidth', style.refLineWidth);
plot(ax3, run.t, run.constraint_flag * max(run.u_s) * 0.9, ':', 'Color', style.red, 'LineWidth', style.lineWidth);
xlabel(ax3, 'Time / s');
ylabel(ax3, 'u_s / V');
title(ax3, 'Voltage magnitude and constraint flag');
legend(ax3, {'u_s', 'u_s,max', 'constraint flag'}, 'Location', 'best');
formatLegend(ax3, style);
formatAxes(ax3, style);
setTimeLimits(ax3, run.t);

saveFigure(fig, p, 'fig_3_11_constraint_projection');
end

function lines = plotMethods(ax, group, fieldName, p, style)
lines = gobjects(numel(p.method_order), 1);
for i = 1:numel(p.method_order)
    method = p.method_order{i};
    run = group.(method);
    lines(i) = plot(ax, run.t, run.(fieldName), ...
        'Color', style.colors(i,:), ...
        'LineWidth', style.lineWidth);
end
end

function h = plotSpeedReference(ax, run, style)
h = plot(ax, run.t, run.omega_ref_rpm, '--', ...
    'Color', style.dark, ...
    'LineWidth', style.refLineWidth);
end

function plotComparison(ax, t1, y1, t2, y2, style)
hold(ax, 'on');
plot(ax, t1, y1, 'Color', style.colors(1,:), 'LineWidth', style.lineWidth);
plot(ax, t2, y2, 'Color', style.colors(2,:), 'LineWidth', style.lineWidth);
end

function labels = methodLabels(p)
labels = {p.method_label.id0, p.method_label.mtpa, p.method_label.mfo};
end

function m = steadyMean(signal, p)
N = numel(signal);
tailStart = max(1, floor((1 - p.metric_tail_fraction) * N));
m = mean(signal(tailStart:end));
end

function fig = newFigure(style, position)
if nargin < 2
    position = [100 100 940 620];
end
fig = figure('Visible', 'off', ...
    'Color', 'w', ...
    'Position', position, ...
    'Renderer', 'painters');
set(fig, ...
    'PaperPositionMode', 'auto', ...
    'DefaultAxesFontName', style.fontName, ...
    'DefaultTextFontName', style.fontName, ...
    'DefaultAxesFontSize', style.fontSize, ...
    'DefaultTextFontSize', style.fontSize);
end

function tl = newTiles(fig, rows, cols)
tl = tiledlayout(fig, rows, cols, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');
end

function formatAxes(ax, style)
grid(ax, 'on');
try
    grid(ax, 'minor');
catch
end
box(ax, 'on');
set(ax, ...
    'FontName', style.fontName, ...
    'FontSize', style.fontSize, ...
    'LineWidth', 0.8, ...
    'TickDir', 'out', ...
    'XColor', style.dark, ...
    'YColor', style.dark, ...
    'GridAlpha', 0.18, ...
    'MinorGridAlpha', 0.08, ...
    'Layer', 'top');
set(get(ax, 'Title'), 'FontWeight', 'bold', 'FontSize', style.fontSize + 1);
set(get(ax, 'XLabel'), 'FontSize', style.fontSize);
set(get(ax, 'YLabel'), 'FontSize', style.fontSize);
end

function formatLegend(ax, style)
try
    legendObj = ax.Legend;
catch
    legendObj = [];
end
if ~isempty(legendObj) && isvalid(legendObj)
    set(legendObj, 'Box', 'off', 'FontName', style.fontName, 'FontSize', style.fontSize - 1);
end
end

function formatFlagAxes(ax, style)
formatAxes(ax, style);
ylim(ax, [-0.08 1.08]);
yticks(ax, [0 1]);
end

function setTimeLimits(ax, t)
if ~isempty(t)
    xlim(ax, [t(1) t(end)]);
end
end

function style = plotStyle()
style.fontName = chooseFont({'Arial', 'Helvetica', 'Times New Roman'});
style.fontSize = 10;
style.lineWidth = 1.65;
style.refLineWidth = 1.2;
style.colors = [
    0.000 0.447 0.698
    0.835 0.369 0.000
    0.000 0.620 0.451
];
style.blue = [0.89 0.95 1.00];
style.green = [0.90 0.97 0.92];
style.orange = [1.00 0.93 0.86];
style.yellow = [0.99 0.96 0.82];
style.dark = [0.12 0.16 0.20];
style.gray = [0.42 0.47 0.52];
style.red = [0.80 0.10 0.10];
end

function fontName = chooseFont(candidates)
fontName = candidates{1};
try
    available = listfonts;
    for i = 1:numel(candidates)
        if any(strcmpi(available, candidates{i}))
            fontName = candidates{i};
            return;
        end
    end
catch
end
end

function flowBox(fig, x, y, w, h, label, fillColor)
annotation(fig, 'textbox', [x y w h], ...
    'String', label, ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'EdgeColor', [0.35 0.40 0.45], ...
    'LineWidth', 1.1, ...
    'BackgroundColor', fillColor, ...
    'FontWeight', 'bold', ...
    'FontSize', 11, ...
    'FitBoxToText', 'off', ...
    'Interpreter', 'none');
end

function flowArrow(fig, xs, ys, color)
annotation(fig, 'arrow', xs, ys, ...
    'Color', color, ...
    'LineWidth', 1.2, ...
    'HeadLength', 8, ...
    'HeadWidth', 8);
end

function saveFigure(fig, p, baseName)
pngPath = fullfile(p.paths.figures, [baseName '.png']);
figPath = fullfile(p.paths.figures, [baseName '.fig']);
drawnow;
if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, pngPath, 'Resolution', 300, 'BackgroundColor', 'white');
else
    print(fig, pngPath, '-dpng', '-r300');
end
if exist('savefig', 'file') == 2
    savefig(fig, figPath);
else
    saveas(fig, figPath);
end
close(fig);
end
