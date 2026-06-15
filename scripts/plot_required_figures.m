function plot_required_figures(allResults, metrics, p)
%PLOT_REQUIRED_FIGURES Export reproducible figures using the Chapter 3 style.

if nargin < 3
    error('plot_required_figures requires allResults, metrics and p.');
end
if exist(p.paths.figures, 'dir') ~= 7
    mkdir(p.paths.figures);
end
if exist(p.paths.chapter_figures, 'dir') ~= 7
    mkdir(p.paths.chapter_figures);
end

style = chapter_style();
plot_overall_model(style, p);
plot_optimizer_subsystem(style, p);
plot_speed_step(allResults.speed_step, style, p);
plot_efficiency_optimization(allResults.efficiency_optimization, style, p);
plot_load_step(allResults.load_step, style, p);
plot_parameter_perturbation(allResults.parameter_perturbation, metrics.parameter_perturbation, style, p);
plot_constraint_test(allResults.constraint_test, style, p);
end

function plot_overall_model(style, p)
fig = new_canvas(style, [2 2 23 10.5]);

box_block(fig, [0.030 0.675 0.100 0.080], 'omega_ref', style.ioFill, style);
box_block(fig, [0.180 0.650 0.155 0.120], 'Speed_Controller_PID', style.moduleFill, style);
box_block(fig, [0.390 0.650 0.165 0.120], 'Current_Controller_PI', style.moduleFill, style);
box_block(fig, [0.620 0.650 0.165 0.120], 'PMSM_dq_Plant', style.moduleFill, style);
box_block(fig, [0.865 0.675 0.090 0.080], 'omega_m', style.ioFill, style);
box_block(fig, [0.642 0.835 0.120 0.070], 'load_torque', style.ioFill, style);
box_block(fig, [0.395 0.455 0.155 0.105], 'Id_Reference_Manager', style.moduleFill, style);
box_block(fig, [0.620 0.430 0.185 0.115], 'Power_Efficiency_Monitor', style.moduleFill, style);
box_block(fig, [0.865 0.445 0.090 0.090], sprintf('id, iq\nP_in, eta'), style.ioFill, style);
box_block(fig, [0.570 0.205 0.165 0.105], 'SteadyState_Detector', style.moduleFill, style);
box_block(fig, [0.345 0.205 0.165 0.105], 'ModelFree_Optimizer', style.moduleFill, style);
box_block(fig, [0.110 0.205 0.185 0.105], 'Projection_or_Safety_Limit', style.moduleFill, style);

text_label(fig, [0.185 0.790 0.600 0.045], 'Main speed/current vector-control loop', style, 'center');
text_label(fig, [0.110 0.125 0.625 0.045], 'Outer model-free power-feedback efficiency optimization loop', style, 'center');

arrow_line(fig, [0.130 0.180], [0.715 0.715], style);
arrow_line(fig, [0.335 0.390], [0.710 0.710], style);
arrow_line(fig, [0.555 0.620], [0.710 0.710], style);
arrow_line(fig, [0.785 0.865], [0.715 0.715], style);
arrow_line(fig, [0.702 0.702], [0.835 0.770], style);
plain_line(fig, [0.835 0.835], [0.715 0.385], style);
plain_line(fig, [0.835 0.255], [0.385 0.385], style);
arrow_line(fig, [0.255 0.255], [0.385 0.650], style);
arrow_line(fig, [0.472 0.472], [0.560 0.650], style);
arrow_line(fig, [0.702 0.702], [0.650 0.545], style);
arrow_line(fig, [0.805 0.865], [0.488 0.488], style);
arrow_line(fig, [0.690 0.690], [0.430 0.310], style);
arrow_line(fig, [0.570 0.510], [0.258 0.258], style);
arrow_line(fig, [0.345 0.295], [0.258 0.258], style);
plain_line(fig, [0.203 0.203], [0.310 0.455], style);
arrow_line(fig, [0.203 0.395], [0.455 0.508], style);

save_outputs(fig, p, '', 'fig3_1_simulink_overall_model', true);
end

function plot_optimizer_subsystem(style, p)
fig = new_canvas(style, [2 2 23 8.8]);

box_block(fig, [0.035 0.490 0.110 0.220], sprintf('Inputs\nu_d, u_q\ni_d, i_q\nomega_m\nsteady_flag'), style.ioFill, style);
box_block(fig, [0.190 0.555 0.125 0.105], sprintf('Input power\ncalculation\nP_in'), style.moduleFill, style);
box_block(fig, [0.360 0.555 0.125 0.105], sprintf('Steady-state\ndecision S(k)'), style.moduleFill, style);
box_block(fig, [0.530 0.555 0.120 0.105], sprintf('Dynamic\nfreeze'), style.moduleFill, style);
box_block(fig, [0.695 0.555 0.120 0.105], sprintf('Perturbation\ninjection'), style.moduleFill, style);
box_block(fig, [0.190 0.285 0.125 0.105], sprintf('Power\ndemodulation'), style.moduleFill, style);
box_block(fig, [0.360 0.285 0.125 0.105], sprintf('Low-pass\nfilter'), style.moduleFill, style);
box_block(fig, [0.530 0.285 0.120 0.105], sprintf('Gradient\nestimation'), style.moduleFill, style);
box_block(fig, [0.695 0.285 0.120 0.105], sprintf('Projection\nupdate'), style.moduleFill, style);
box_block(fig, [0.850 0.285 0.095 0.105], sprintf('Rate\nlimit'), style.moduleFill, style);
box_block(fig, [0.850 0.500 0.105 0.185], sprintf('Outputs\ni_d^*\noptimizer_active\nconstraint_flag'), style.ioFill, style);

text_label(fig, [0.060 0.790 0.880 0.045], 'Outer model-free optimizer subsystem logic', style, 'center');
text_label(fig, [0.310 0.715 0.470 0.045], 'steady_flag = 0: freeze and hold i_d^*    steady_flag = 1: slow update by P_in trend', style, 'center');
text_label(fig, [0.690 0.175 0.250 0.045], 'Current, voltage and safety boundary projection', style, 'center');

arrow_line(fig, [0.145 0.190], [0.600 0.600], style);
arrow_line(fig, [0.315 0.360], [0.608 0.608], style);
arrow_line(fig, [0.485 0.530], [0.608 0.608], style);
arrow_line(fig, [0.650 0.695], [0.608 0.608], style);
arrow_line(fig, [0.815 0.850], [0.600 0.600], style);
plain_line(fig, [0.252 0.252], [0.555 0.390], style);
arrow_line(fig, [0.252 0.190], [0.390 0.338], style);
arrow_line(fig, [0.315 0.360], [0.338 0.338], style);
arrow_line(fig, [0.485 0.530], [0.338 0.338], style);
arrow_line(fig, [0.650 0.695], [0.338 0.338], style);
arrow_line(fig, [0.815 0.850], [0.338 0.338], style);
arrow_line(fig, [0.902 0.902], [0.390 0.510], style);
plain_line(fig, [0.590 0.590], [0.555 0.455], style);
arrow_line(fig, [0.590 0.755], [0.455 0.390], style);
text_label(fig, [0.603 0.468 0.155 0.035], 'hold during dynamic state', style, 'center');

save_outputs(fig, p, '', 'fig3_2_model_free_optimizer_subsystem', true);
end

function plot_speed_step(group, style, p)
fig = new_figure(style, [2 2 16 10.5]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl);
hold(ax1, 'on');
plot(ax1, group.baseline.t, rpm(group.baseline.omega_ref), '--', 'Color', style.refColor, 'LineWidth', 1.4);
plot(ax1, group.baseline.t, group.baseline.omega_rpm, '-', 'Color', style.pidColor, 'LineWidth', 1.8);
plot(ax1, group.optimization.t, group.optimization.omega_rpm, '-', 'Color', style.methodColor, 'LineWidth', 1.8);
xlabel(ax1, 't / s', 'Interpreter', 'tex');
ylabel(ax1, 'n / r\cdotmin^{-1}', 'Interpreter', 'tex');
legend(ax1, {'Reference speed', 'i_d^*=0', 'Model-free optimization'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax1, style);
set_time_limits(ax1, group.baseline.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
plot(ax2, group.baseline.t, group.baseline.iq, '-', 'Color', style.pidColor, 'LineWidth', 1.55);
plot(ax2, group.optimization.t, group.optimization.iq, '-', 'Color', style.methodColor, 'LineWidth', 1.55);
plot(ax2, group.optimization.t, group.optimization.id_ref, '-', 'Color', style.greenColor, 'LineWidth', 1.25);
xlabel(ax2, 't / s', 'Interpreter', 'tex');
ylabel(ax2, 'Current / A', 'Interpreter', 'tex');
legend(ax2, {'i_q baseline', 'i_q proposed', 'i_d^* proposed'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax2, style);
set_time_limits(ax2, group.baseline.t);

save_outputs(fig, p, 'fig_speed_step', 'fig3_3_speed_step_response', false);
end

function plot_efficiency_optimization(group, style, p)
fig = new_figure(style, [2 2 16 10.5]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl);
yyaxis(ax1, 'left');
plot(ax1, group.optimization.t, group.optimization.Pin, '-', 'Color', style.pidColor, 'LineWidth', 1.75);
ylabel(ax1, 'P_{in} / W', 'Interpreter', 'tex');
yyaxis(ax1, 'right');
plot(ax1, group.optimization.t, group.optimization.id_bar, '-', 'Color', style.methodColor, 'LineWidth', 1.75);
ylabel(ax1, 'i_{d,bar}^* / A', 'Interpreter', 'tex');
xlabel(ax1, 't / s', 'Interpreter', 'tex');
format_axes(ax1, style);
format_yyaxis(ax1, style);
set_time_limits(ax1, group.optimization.t);
legend(ax1, {'Input power P_{in}', 'Mean d-axis current i_{d,bar}^*'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');

ax2 = nexttile(tl);
hold(ax2, 'on');
plot(ax2, group.optimization.t, group.optimization.g_hat, '-', 'Color', style.greenColor, 'LineWidth', 1.45);
stairs(ax2, group.optimization.t, group.optimization.optimizer_enable, '-', 'Color', style.axisColor, 'LineWidth', 1.05);
stairs(ax2, group.optimization.t, group.optimization.projection_active, '--', 'Color', style.eventColor, 'LineWidth', 1.05);
xlabel(ax2, 't / s', 'Interpreter', 'tex');
ylabel(ax2, 'g_hat / state', 'Interpreter', 'none');
legend(ax2, {'Gradient estimate', 'optimizer enable', 'projection active'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax2, style);
set_time_limits(ax2, group.optimization.t);

save_outputs(fig, p, 'fig_efficiency_optimization', 'fig3_4_power_id_convergence', false);
end

function plot_load_step(group, style, p)
fig = new_figure(style, [2 2 16 11]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl);
hold(ax1, 'on');
plot(ax1, group.baseline.t, rpm(group.baseline.omega_ref), '--', 'Color', style.refColor, 'LineWidth', 1.4);
plot(ax1, group.baseline.t, group.baseline.omega_rpm, '-', 'Color', style.pidColor, 'LineWidth', 1.75);
plot(ax1, group.optimization.t, group.optimization.omega_rpm, '-', 'Color', style.methodColor, 'LineWidth', 1.75);
add_event_line(ax1, p.exp.load_step.t_step, style);
ylabel(ax1, 'n / r\cdotmin^{-1}', 'Interpreter', 'tex');
legend(ax1, {'Reference speed', 'i_d^*=0', 'Model-free optimization'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax1, style);
set_time_limits(ax1, group.baseline.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
plot(ax2, group.optimization.t, group.optimization.T_L, '-', 'Color', style.pidColor, 'LineWidth', 1.55);
plot(ax2, group.optimization.t, group.optimization.id_bar, '-', 'Color', style.methodColor, 'LineWidth', 1.55);
stairs(ax2, group.optimization.t, group.optimization.steady_flag, '-', 'Color', style.greenColor, 'LineWidth', 1.2);
stairs(ax2, group.optimization.t, group.optimization.freeze_state, '--', 'Color', style.eventColor, 'LineWidth', 1.0);
add_event_line(ax2, p.exp.load_step.t_step, style);
xlabel(ax2, 't / s', 'Interpreter', 'tex');
ylabel(ax2, 'T_L / i_{d,bar}^* / state', 'Interpreter', 'tex');
legend(ax2, {'Load torque', 'i_{d,bar}^*', 'steady flag', 'freeze state'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax2, style);
set_time_limits(ax2, group.optimization.t);

save_outputs(fig, p, 'fig_load_step', 'fig3_5_disturbance_response', false);
end

function plot_parameter_perturbation(group, metricGroup, style, p)
fig = new_figure(style, [2 2 16 10.5]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl);
hold(ax1, 'on');
plot(ax1, group.nominal.optimization.t, group.nominal.optimization.id_bar, '-', 'Color', style.methodColor, 'LineWidth', 1.65);
plot(ax1, group.perturbed.optimization.t, group.perturbed.optimization.id_bar, '-', 'Color', style.greenColor, 'LineWidth', 1.65);
xlabel(ax1, 't / s', 'Interpreter', 'tex');
ylabel(ax1, 'i_{d,bar}^* / A', 'Interpreter', 'tex');
legend(ax1, {'Nominal plant', 'Perturbed plant'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax1, style);
set_time_limits(ax1, group.nominal.optimization.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
vals = [metricGroup.nominal.mean_Pin_W; metricGroup.perturbed.mean_Pin_W];
labels = categorical({'nominal baseline'; 'nominal optimized'; 'perturbed baseline'; 'perturbed optimized'});
labels = reordercats(labels, cellstr(labels));
bar(ax2, labels, vals, 'FaceColor', style.barColor, 'EdgeColor', style.axisColor, 'LineWidth', 0.8);
ylabel(ax2, 'Mean P_{in} / W', 'Interpreter', 'tex');
format_axes(ax2, style);

save_outputs(fig, p, 'fig_parameter_perturbation', '', false);
end

function plot_constraint_test(group, style, p)
fig = new_figure(style, [2 2 16 11]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl);
hold(ax1, 'on');
plot(ax1, group.optimization.t, group.optimization.is, '-', 'Color', style.pidColor, 'LineWidth', 1.55);
plot(ax1, group.optimization.t, p.Imax * ones(size(group.optimization.t)), '--', 'Color', style.refColor, 'LineWidth', 1.15);
ylabel(ax1, 'i_s / A', 'Interpreter', 'tex');
legend(ax1, {'Stator current', 'Current limit'}, 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax1, style);
set_time_limits(ax1, group.optimization.t);

ax2 = nexttile(tl);
hold(ax2, 'on');
plot(ax2, group.optimization.t, group.optimization.us, '-', 'Color', style.methodColor, 'LineWidth', 1.55);
plot(ax2, group.optimization.t, p.Vmax * ones(size(group.optimization.t)), '--', 'Color', style.refColor, 'LineWidth', 1.15);
ylabel(ax2, 'u_s / V', 'Interpreter', 'tex');
legend(ax2, {'Voltage vector', 'Voltage limit'}, 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax2, style);
set_time_limits(ax2, group.optimization.t);

ax3 = nexttile(tl);
hold(ax3, 'on');
stairs(ax3, group.optimization.t, group.optimization.projection_active, '-', 'Color', style.pidColor, 'LineWidth', 1.2);
stairs(ax3, group.optimization.t, group.optimization.freeze_state, '-', 'Color', style.methodColor, 'LineWidth', 1.2);
stairs(ax3, group.optimization.t, group.optimization.current_saturated, '--', 'Color', style.greenColor, 'LineWidth', 1.0);
stairs(ax3, group.optimization.t, group.optimization.voltage_saturated, '--', 'Color', style.eventColor, 'LineWidth', 1.0);
xlabel(ax3, 't / s', 'Interpreter', 'tex');
ylabel(ax3, 'state', 'Interpreter', 'none');
ylim(ax3, [-0.08 1.08]);
legend(ax3, {'projection', 'freeze', 'current sat.', 'voltage sat.'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
format_axes(ax3, style);
set_time_limits(ax3, group.optimization.t);

save_outputs(fig, p, 'fig_constraint_test', '', false);
end

function y = rpm(omega)
y = omega * 60 / (2*pi);
end

function fig = new_canvas(style, positionCm)
fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', ...
    'Position', positionCm, 'PaperPositionMode', 'auto', 'Renderer', 'painters');
set(fig, 'DefaultAxesFontName', style.fontName, 'DefaultTextFontName', style.fontName, ...
    'DefaultAxesFontSize', style.figureFontSize, 'DefaultTextFontSize', style.figureFontSize);
axes('Parent', fig, 'Position', [0 0 1 1], 'Visible', 'off');
end

function fig = new_figure(style, positionCm)
fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', ...
    'Position', positionCm, 'PaperPositionMode', 'auto', 'Renderer', 'painters');
set(fig, 'DefaultAxesFontName', style.fontName, 'DefaultTextFontName', style.fontName, ...
    'DefaultAxesFontSize', style.figureFontSize, 'DefaultTextFontSize', style.figureFontSize);
end

function format_axes(ax, style)
grid(ax, 'on');
box(ax, 'on');
set(ax, ...
    'FontName', style.fontName, ...
    'FontSize', style.figureFontSize, ...
    'LineWidth', 0.9, ...
    'XColor', style.axisColor, ...
    'YColor', style.axisColor, ...
    'GridColor', style.gridColor, ...
    'GridAlpha', 0.35, ...
    'TickDir', 'out', ...
    'Layer', 'top');
try
    set(ax.Legend, 'FontName', style.fontName, 'FontSize', style.figureFontSize - 1);
catch
end
end

function format_yyaxis(ax, style)
try
    ax.YAxis(1).Color = style.axisColor;
    ax.YAxis(2).Color = style.axisColor;
catch
end
end

function set_time_limits(ax, t)
if ~isempty(t)
    xlim(ax, [min(t) max(t)]);
end
end

function add_event_line(ax, x, style)
yl = ylim(ax);
plot(ax, [x x], yl, '--', 'Color', style.eventColor, 'LineWidth', 1.15);
ylim(ax, yl);
text(ax, x + 0.04, yl(2) - 0.10 * diff(yl), 't = 2 s', ...
    'FontName', style.fontName, 'FontSize', style.figureFontSize - 1, ...
    'Color', style.eventColor, 'Interpreter', 'none');
end

function box_block(fig, pos, label, fillColor, style)
annotation(fig, 'textbox', pos, ...
    'String', label, ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FontName', style.fontName, ...
    'FontSize', style.fontSize, ...
    'FontWeight', 'normal', ...
    'Interpreter', 'none', ...
    'EdgeColor', style.edgeColor, ...
    'LineWidth', 1.0, ...
    'BackgroundColor', fillColor, ...
    'FitBoxToText', 'off', ...
    'Margin', 4);
end

function text_label(fig, pos, label, style, align)
annotation(fig, 'textbox', pos, ...
    'String', label, ...
    'HorizontalAlignment', align, ...
    'VerticalAlignment', 'middle', ...
    'FontName', style.fontName, ...
    'FontSize', style.fontSize, ...
    'Interpreter', 'none', ...
    'EdgeColor', 'none', ...
    'BackgroundColor', 'none', ...
    'FitBoxToText', 'off', ...
    'Color', style.axisColor);
end

function arrow_line(fig, xs, ys, style)
annotation(fig, 'arrow', xs, ys, ...
    'Color', style.axisColor, ...
    'LineWidth', 1.15, ...
    'HeadLength', 8, ...
    'HeadWidth', 8);
end

function plain_line(fig, xs, ys, style)
annotation(fig, 'line', xs, ys, ...
    'Color', style.axisColor, ...
    'LineWidth', 1.05);
end

function save_outputs(fig, p, requiredName, chapterName, alsoSvg)
drawnow;
if ~isempty(requiredName)
    save_one(fig, fullfile(p.paths.figures, [requiredName '.png']));
    save_figure_file(fig, fullfile(p.paths.figures, [requiredName '.fig']));
end
if ~isempty(chapterName)
    save_one(fig, fullfile(p.paths.chapter_figures, [chapterName '.png']));
    save_figure_file(fig, fullfile(p.paths.chapter_figures, [chapterName '.fig']));
    if alsoSvg
        print(fig, fullfile(p.paths.chapter_figures, [chapterName '.svg']), '-dsvg', '-r300');
    end
end
close(fig);
end

function save_one(fig, path)
if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, path, 'Resolution', 300, 'BackgroundColor', 'white');
else
    print(fig, path, '-dpng', '-r300');
end
end

function save_figure_file(fig, path)
if exist('savefig', 'file') == 2
    savefig(fig, path);
else
    saveas(fig, path);
end
end

function style = chapter_style()
style.fontName = choose_font({'Times New Roman', 'SimSun', 'Arial'});
style.fontSize = 8.5;
style.figureFontSize = 10;
style.axisColor = [0.05 0.05 0.05];
style.edgeColor = [0.20 0.20 0.20];
style.gridColor = [0.78 0.78 0.78];
style.refColor = [0.15 0.15 0.15];
style.pidColor = [0.55 0.18 0.16];
style.methodColor = [0.00 0.32 0.55];
style.greenColor = [0.00 0.45 0.32];
style.eventColor = [0.25 0.25 0.25];
style.barColor = [0.72 0.80 0.88];
style.moduleFill = [0.96 0.96 0.96];
style.ioFill = [1.00 1.00 1.00];
end

function fontName = choose_font(candidates)
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
