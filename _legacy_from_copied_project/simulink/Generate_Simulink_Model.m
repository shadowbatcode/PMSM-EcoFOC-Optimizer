function modelPath = Generate_Simulink_Model(p)
%GENERATE_SIMULINK_MODEL Build an editable Simulink model matching the script architecture.
%
% The generated model is an architecture-level companion to the MATLAB
% simulation code. The numeric reference implementation remains
% src/Simulate_PMSM_FOC.m and the controller/model functions it calls.

if nargin < 1 || isempty(p)
    srcDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src');
    addpath(srcDir);
    p = Parameter_Set();
end

modelName = 'PMSM_FOC_Model';
targetPath = fullfile(p.paths.simulink, [modelName '.slx']);
modelPath = '';

if isempty(ver('simulink')) || ~license('test', 'Simulink')
    warning('PMSM:NoSimulink', 'Simulink is not available. Model file was not generated.');
    return;
end
if ~hasEnoughMemory()
    warning('PMSM:LowMemoryForSimulink', ...
        'Available physical memory is low. Skipped Simulink model generation to avoid MATLAB/Java instability.');
    return;
end

if ~exist(p.paths.simulink, 'dir')
    mkdir(p.paths.simulink);
end

if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end

try
    new_system(modelName);
    set_param(modelName, ...
        'Solver', 'FixedStepDiscrete', ...
        'FixedStep', num2str(p.Ts), ...
        'StopTime', num2str(p.Tsim), ...
        'SampleTimeColors', 'on', ...
        'ShowPortDataTypes', 'on');

    addTopLevelBlocks(modelName);
    addSubsystemDefinitions(modelName);
    addTopLevelLines(modelName);
    addModelNotes(modelName, p);

    save_system(modelName, targetPath);
    close_system(modelName, 0);
catch ME
    if bdIsLoaded(modelName)
        set_param(modelName, 'Dirty', 'off');
        close_system(modelName, 0);
    end
    rethrow(ME);
end

modelPath = targetPath;
fprintf('Simulink model generated: %s\n', modelPath);
end

function ok = hasEnoughMemory()
ok = true;
if ispc && exist('memory', 'file') == 2
    try
        [~, systemView] = memory;
        ok = systemView.PhysicalMemory.Available >= 3.0e9;
    catch
        ok = true;
    end
end
end

function addTopLevelBlocks(modelName)
add_block('built-in/Inport', [modelName '/omega_ref'], ...
    'Position', [40 90 70 110]);
add_block('built-in/Inport', [modelName '/load_torque'], ...
    'Position', [40 350 70 370]);

addSubsystem(modelName, 'Speed_Controller_PID', [140 60 310 160], 'cyan');
addSubsystem(modelName, 'Id_Reference_Manager', [375 50 575 175], 'green');
addSubsystem(modelName, 'Current_Controller_PI', [650 70 850 190], 'yellow');
addSubsystem(modelName, 'PMSM_dq_Plant', [930 95 1130 245], 'magenta');
addSubsystem(modelName, 'Power_Efficiency_Monitor', [930 330 1130 450], 'cyan');
addSubsystem(modelName, 'SteadyState_Detector', [375 255 575 345], 'green');
addSubsystem(modelName, 'ModelFree_Optimizer', [375 405 575 505], 'green');
addSubsystem(modelName, 'Projection_Omega', [650 405 850 505], 'yellow');
add_block('built-in/Mux', [modelName '/id_iq_mux'], ...
    'Inputs', '2', ...
    'Position', [1160 172 1170 205]);

add_block('built-in/Outport', [modelName '/omega_m'], ...
    'Position', [1210 120 1240 140]);
add_block('built-in/Outport', [modelName '/id_iq'], ...
    'Position', [1210 175 1240 195]);
add_block('built-in/Outport', [modelName '/P_in'], ...
    'Position', [1210 365 1240 385]);
add_block('built-in/Outport', [modelName '/eta'], ...
    'Position', [1210 420 1240 440]);
end

function addSubsystem(modelName, name, position, color)
add_block('built-in/SubSystem', [modelName '/' name], ...
    'Position', position, ...
    'BackgroundColor', color, ...
    'ShowName', 'on');
end

function addSubsystemDefinitions(modelName)
configureSubsystem([modelName '/Speed_Controller_PID'], ...
    {'omega_ref', 'omega_m'}, {'iq_ref', 'speed_error'}, ...
    'src/Speed_Controller_PID.m');
configureSubsystem([modelName '/Id_Reference_Manager'], ...
    {'iq_ref', 'P_in', 'id_bar_projected', 'omega_ref', 'omega_m'}, {'id_ref', 'optimizer_active'}, ...
    'src/MTPA_Controller.m + src/ModelFree_Optimizer.m');
configureSubsystem([modelName '/Current_Controller_PI'], ...
    {'id_ref', 'iq_ref', 'id', 'iq', 'omega_e'}, {'u_d', 'u_q', 'u_s', 'voltage_limited'}, ...
    'src/Current_Controller_PI.m');
configureSubsystem([modelName '/PMSM_dq_Plant'], ...
    {'u_d', 'u_q', 'T_L'}, {'id', 'iq', 'omega_m', 'omega_e', 'T_e'}, ...
    'src/PMSM_Model.m');
configureSubsystem([modelName '/Power_Efficiency_Monitor'], ...
    {'u_d', 'u_q', 'id', 'iq', 'omega_m', 'T_e'}, {'P_in', 'Pbar_in', 'eta'}, ...
    'src/Simulate_PMSM_FOC.m logging equations');
configureSubsystem([modelName '/SteadyState_Detector'], ...
    {'omega_ref', 'omega_m', 'iq_ref', 'iq', 'Pbar_in'}, {'steady_flag'}, ...
    'src/SteadyState_Detector.m');
configureSubsystem([modelName '/ModelFree_Optimizer'], ...
    {'P_in', 'iq_ref', 'u_s', 'steady_flag'}, {'id_bar_candidate', 'id_bar_projected'}, ...
    'src/ModelFree_Optimizer.m');
configureSubsystem([modelName '/Projection_Omega'], ...
    {'id_bar_candidate', 'iq_ref', 'u_s'}, {'id_bar_projected', 'constraint_flag'}, ...
    'src/Projection_Omega.m');
end

function configureSubsystem(systemPath, inputs, outputs, noteText)
removeSubsystemContents(systemPath);

for i = 1:numel(inputs)
    y = 30 + (i - 1) * 32;
    add_block('built-in/Inport', [systemPath '/' inputs{i}], ...
        'Position', [35 y 65 y + 14]);
end

for i = 1:numel(outputs)
    y = 30 + (i - 1) * 32;
    add_block('built-in/Outport', [systemPath '/' outputs{i}], ...
        'Position', [255 y 285 y + 14]);
end

try
    note = Simulink.Annotation(systemPath, sprintf('Code reference:\n%s', noteText));
    note.Position = [95 35 235 95];
    note.FontSize = 10;
    note.Interpreter = 'none';
catch
end
end

function removeSubsystemContents(systemPath)
lines = find_system(systemPath, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for i = 1:numel(lines)
    delete_line(lines(i));
end

blocks = find_system(systemPath, 'SearchDepth', 1, 'Type', 'Block');
for i = 1:numel(blocks)
    if ~strcmp(blocks{i}, systemPath)
        delete_block(blocks{i});
    end
end
end

function addTopLevelLines(modelName)
line(modelName, 'omega_ref/1', 'Speed_Controller_PID/1');
line(modelName, 'Speed_Controller_PID/1', 'Current_Controller_PI/2');
line(modelName, 'Speed_Controller_PID/1', 'Id_Reference_Manager/1');
line(modelName, 'omega_ref/1', 'Id_Reference_Manager/4');
line(modelName, 'Id_Reference_Manager/1', 'Current_Controller_PI/1');
line(modelName, 'Current_Controller_PI/1', 'PMSM_dq_Plant/1');
line(modelName, 'Current_Controller_PI/2', 'PMSM_dq_Plant/2');
line(modelName, 'load_torque/1', 'PMSM_dq_Plant/3');

line(modelName, 'PMSM_dq_Plant/1', 'Current_Controller_PI/3');
line(modelName, 'PMSM_dq_Plant/2', 'Current_Controller_PI/4');
line(modelName, 'PMSM_dq_Plant/3', 'Speed_Controller_PID/2');
line(modelName, 'PMSM_dq_Plant/3', 'Id_Reference_Manager/5');
line(modelName, 'PMSM_dq_Plant/4', 'Current_Controller_PI/5');

line(modelName, 'Current_Controller_PI/1', 'Power_Efficiency_Monitor/1');
line(modelName, 'Current_Controller_PI/2', 'Power_Efficiency_Monitor/2');
line(modelName, 'PMSM_dq_Plant/1', 'Power_Efficiency_Monitor/3');
line(modelName, 'PMSM_dq_Plant/2', 'Power_Efficiency_Monitor/4');
line(modelName, 'PMSM_dq_Plant/3', 'Power_Efficiency_Monitor/5');
line(modelName, 'PMSM_dq_Plant/5', 'Power_Efficiency_Monitor/6');

line(modelName, 'omega_ref/1', 'SteadyState_Detector/1');
line(modelName, 'PMSM_dq_Plant/3', 'SteadyState_Detector/2');
line(modelName, 'Speed_Controller_PID/1', 'SteadyState_Detector/3');
line(modelName, 'PMSM_dq_Plant/2', 'SteadyState_Detector/4');
line(modelName, 'Power_Efficiency_Monitor/2', 'SteadyState_Detector/5');

line(modelName, 'Power_Efficiency_Monitor/1', 'Id_Reference_Manager/2');
line(modelName, 'Power_Efficiency_Monitor/1', 'ModelFree_Optimizer/1');
line(modelName, 'Speed_Controller_PID/1', 'ModelFree_Optimizer/2');
line(modelName, 'Current_Controller_PI/3', 'ModelFree_Optimizer/3');
line(modelName, 'SteadyState_Detector/1', 'ModelFree_Optimizer/4');
line(modelName, 'ModelFree_Optimizer/1', 'Projection_Omega/1');
line(modelName, 'Speed_Controller_PID/1', 'Projection_Omega/2');
line(modelName, 'Current_Controller_PI/3', 'Projection_Omega/3');
line(modelName, 'Projection_Omega/1', 'Id_Reference_Manager/3');

line(modelName, 'PMSM_dq_Plant/3', 'omega_m/1');
line(modelName, 'PMSM_dq_Plant/1', 'id_iq_mux/1');
line(modelName, 'PMSM_dq_Plant/2', 'id_iq_mux/2');
line(modelName, 'id_iq_mux/1', 'id_iq/1');
line(modelName, 'Power_Efficiency_Monitor/1', 'P_in/1');
line(modelName, 'Power_Efficiency_Monitor/3', 'eta/1');
end

function line(modelName, fromPort, toPort)
try
    add_line(modelName, fromPort, toPort, 'autorouting', 'on');
catch ME
    warning('PMSM:LineSkipped', 'Skipped line %s -> %s: %s', fromPort, toPort, ME.message);
end
end

function addModelNotes(modelName, p)
try
    titleNote = Simulink.Annotation(modelName, ...
        sprintf('PMSM FOC energy-optimization architecture\nTs = %.1g s, Tsim = %.1f s', p.Ts, p.Tsim));
    titleNote.Position = [365 5 805 45];
    titleNote.FontSize = 13;
    titleNote.FontWeight = 'bold';

    flowNote = Simulink.Annotation(modelName, ...
        'Main loop: speed PID -> d/q current PI -> voltage limit -> PMSM dq plant. Feedback loop: input power and steady-state detector drive MTPA/MFO id reference selection.');
    flowNote.Position = [245 560 1050 605];
    flowNote.FontSize = 10;
    flowNote.Interpreter = 'none';
catch
end
end
