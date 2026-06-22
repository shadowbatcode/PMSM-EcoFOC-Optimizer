if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end

beautify_one_component_model(fullfile(p.paths.models, 'PMSM_FOC_Component_Baseline.slx'), 'baseline', p);
beautify_one_component_model(fullfile(p.paths.models, 'PMSM_FOC_Component_Optimization.slx'), 'optimization', p);

function beautify_one_component_model(modelPath, method, p)
[~, modelName] = fileparts(modelPath);
load_system(modelPath);
open_system(modelName);

set_param(modelName, ...
    'ScreenColor', 'white', ...
    'ShowGrid', 'on', ...
    'GridSpacing', '20', ...
    'ZoomFactor', 'FitSystem');

palette.io = '[0.88 0.94 1.00]';
palette.main = '[0.88 0.97 0.90]';
palette.opt = '[1.00 0.93 0.78]';
palette.proj = '[1.00 0.88 0.82]';
palette.plant = '[1.00 0.97 0.78]';
palette.monitor = '[0.86 0.96 0.96]';
palette.state = '[0.93 0.93 0.93]';
palette.sink = '[1.00 1.00 1.00]';

set_block(modelName, 'omega_ref', [40 88 90 118], palette.io);
set_block(modelName, 'T_L', [40 210 90 240], palette.io);
set_block(modelName, 't', [40 330 90 360], palette.io);
set_block(modelName, 'method_flag', [120 330 190 360], palette.io);

set_block(modelName, 'Speed_Controller_PID', [155 65 325 145], palette.main);
set_block(modelName, 'ModelFree_Optimizer', [380 245 610 490], palette.opt);
set_block(modelName, 'Safety_Projection', [675 150 835 250], palette.proj);
set_block(modelName, 'Current_Controller_PI', [890 65 1085 195], palette.main);
set_block(modelName, 'PMSM_dq_Plant', [1140 65 1335 215], palette.plant);
set_block(modelName, 'Power_Efficiency_Monitor', [1390 95 1605 205], palette.monitor);

stateNames = {'z_id','z_iq','z_omega_m','z_speed_int','z_id_int','z_iq_int', ...
    'z_Pin','z_Pin_prev','z_Pin_lpf','z_g_hat_lpf','z_id_bar','z_u_s', ...
    'z_id_ref_prev','z_steady_count','z_perturb_ramp','z_current_sat','z_voltage_sat'};
for k = 1:numel(stateNames)
    col = mod(k-1, 9);
    row = floor((k-1) / 9);
    x = 155 + col * 124;
    y = 560 + row * 46;
    set_block(modelName, stateNames{k}, [x y x+96 y+24], palette.state);
end

names = pmsm_foc_output_names();
for k = 1:numel(names)
    col = floor((k-1) / 12);
    row = mod(k-1, 12);
    x = 1680 + col * 132;
    y = 70 + row * 34;
    set_block(modelName, names{k}, [x y x+112 y+22], palette.sink);
end

name_lines(modelName);
replace_annotations(modelName, method, p);
style_lines(modelName);
route_lines(modelName);

try
    set_param(modelName, 'ZoomFactor', 'FitSystem');
catch
end
set_param(modelName, 'ZoomFactor', 'FitSystem');
save_system(modelName, modelPath);
set_param(modelName, 'Dirty', 'off');
close_system(modelName, 0);
end

function set_block(modelName, blockName, pos, color)
path = [modelName '/' blockName];
if getSimulinkBlockHandle(path) <= 0
    return;
end
set_param(path, 'Position', pos);
try
    set_param(path, 'BackgroundColor', color);
catch
end
try
    set_param(path, 'ForegroundColor', 'black');
catch
end
try
    set_param(path, 'DropShadow', 'on');
catch
end
end

function name_lines(modelName)
lineSpecs = {
    'omega_ref', 'omega_ref'
    'T_L', 'load_torque'
    'Speed_Controller_PID', 'iq_ref_raw'
    'ModelFree_Optimizer', 'id_ref_raw'
    'Safety_Projection', 'id_ref'
    'Current_Controller_PI', 'u_dq'
    'PMSM_dq_Plant', 'plant_states'
    'Power_Efficiency_Monitor', 'power_eta'
    };
for k = 1:size(lineSpecs, 1)
    src = [modelName '/' lineSpecs{k, 1}];
    if getSimulinkBlockHandle(src) <= 0
        continue;
    end
    ph = get_param(src, 'PortHandles');
    if isempty(ph.Outport)
        continue;
    end
    try
        line = get_param(ph.Outport(1), 'Line');
        if line > 0
            set_param(line, 'Name', lineSpecs{k, 2});
        end
    catch
    end
end
end

function replace_annotations(modelName, method, p)
old = find_system(modelName, 'FindAll', 'on', 'Type', 'annotation');
for k = 1:numel(old)
    try
        delete(old(k));
    catch
    end
end

modeLabel = upper(method);
if strcmp(method, 'optimization')
    modeLabel = 'PROPOSED MODEL-FREE OPTIMIZATION';
else
    modeLabel = 'CONVENTIONAL PID/PI BASELINE';
end

add_band(modelName, [20 45 1645 520], [0.97 0.99 1.00], 'Main PMSM vector-control and energy-optimization signal flow');
add_band(modelName, [140 530 1295 665], [0.98 0.98 0.98], 'Discrete state memory bank below the modules');
add_band(modelName, [1660 45 1960 500], [0.99 0.99 0.99], 'Logged outputs');

title = Simulink.Annotation(modelName, sprintf('PMSM PID/PI Vector Control with Energy Self-Optimization  |  %s', modeLabel));
title.Position = [410 5 1460 38];
title.FontSize = 15;
title.FontWeight = 'bold';

subtitle = Simulink.Annotation(modelName, sprintf('MATLAB/Simulink R2025b visible component implementation | Ts = %.1e s | dq PMSM plant + PID/PI loops + power-feedback optimizer', p.Ts));
subtitle.Position = [350 36 1580 62];
subtitle.FontSize = 10;

main = Simulink.Annotation(modelName, 'Fast inner loop: speed PID/PI -> current PI -> dq PMSM electrical/mechanical plant');
main.Position = [155 160 1335 190];
main.FontSize = 10;
main.ForegroundColor = '[0.00 0.25 0.50]';

outer = Simulink.Annotation(modelName, 'Slow outer loop: steady-state decision -> power demodulation -> d-axis reference update -> safety projection');
outer.Position = [380 500 1260 528];
outer.FontSize = 10;
outer.ForegroundColor = '[0.55 0.24 0.00]';

state = Simulink.Annotation(modelName, 'Unit Delay states: controller integrators, plant states, filtered power, optimizer memory and protection flags');
state.Position = [155 505 1120 530];
state.FontSize = 10;
state.FontWeight = 'bold';

outputs = Simulink.Annotation(modelName, 'Paper-aligned logged signals');
outputs.Position = [1680 45 1960 70];
outputs.FontSize = 10;
outputs.FontWeight = 'bold';
end

function add_band(modelName, pos, color, label)
ann = Simulink.Annotation(modelName, label);
ann.Position = pos;
ann.BackgroundColor = sprintf('[%.3f %.3f %.3f]', color);
ann.ForegroundColor = '[0.35 0.35 0.35]';
ann.FontSize = 9;
try
    ann.DropShadow = 'off';
catch
end
end

function style_lines(modelName)
lines = find_system(modelName, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(lines)
    try
        set_param(lines(k), 'LineWidth', 1.5);
    catch
    end
end
highlight_line_from(modelName, 'omega_ref', '[0.10 0.10 0.10]');
highlight_line_from(modelName, 'Speed_Controller_PID', '[0.00 0.28 0.55]');
highlight_line_from(modelName, 'ModelFree_Optimizer', '[0.75 0.30 0.10]');
highlight_line_from(modelName, 'Safety_Projection', '[0.75 0.30 0.10]');
highlight_line_from(modelName, 'Current_Controller_PI', '[0.00 0.28 0.55]');
highlight_line_from(modelName, 'PMSM_dq_Plant', '[0.10 0.40 0.20]');
highlight_line_from(modelName, 'Power_Efficiency_Monitor', '[0.00 0.45 0.45]');
end

function highlight_line_from(modelName, blockName, color)
path = [modelName '/' blockName];
if getSimulinkBlockHandle(path) <= 0
    return;
end
ph = get_param(path, 'PortHandles');
for k = 1:numel(ph.Outport)
    try
        line = get_param(ph.Outport(k), 'Line');
        if line > 0
            set_param(line, 'ForegroundColor', color, 'LineWidth', 2.0);
        end
    catch
    end
end
end

function route_lines(modelName)
lines = find_system(modelName, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(lines)
    try
        Simulink.BlockDiagram.routeLine(lines(k));
    catch
    end
end
end
