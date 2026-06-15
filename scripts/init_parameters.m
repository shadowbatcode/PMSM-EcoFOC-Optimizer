%INIT_PARAMETERS Centralized parameters for the reproducible PMSM project.

project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'src'));

p = struct();
p.project_root = project_root;
p.paths.root = p.project_root;
p.paths.src = fullfile(p.project_root, 'src');
p.paths.models = fullfile(p.project_root, 'models');
p.paths.scripts = fullfile(p.project_root, 'scripts');
p.paths.results = fullfile(p.project_root, 'results');
p.paths.figures = fullfile(p.project_root, 'figures');
p.paths.chapter_figures = fullfile(p.project_root, 'figures_chapter3');
p.paths.data = fullfile(p.project_root, 'results', 'data');
p.paths.tables = fullfile(p.project_root, 'results', 'tables');
p.paths.logs = fullfile(p.project_root, 'results', 'logs');
p.paths.simulink = p.paths.models;
p.paths.legacy = fullfile(p.project_root, '_legacy_from_copied_project');

% Plant parameters from the paper/task statement.
p.Rs = 0.8;
p.Ld = 3.5e-3;
p.Lq = 8.5e-3;
p.psi_f = 0.105;
p.pole_pairs = 4;
p.J = 2.0e-3;
p.B = 5.0e-4;
p.Vdc = 300;
p.Vmax = p.Vdc / sqrt(3);
p.Ts = 1.0e-4;
p.Tstop = 4.0;

% Limits and protection thresholds.
p.Imax = 25;
p.id_min = -18;
p.id_max = 4;
p.iq_max = 24;
p.speed_int_limit = 80;
p.current_int_limit = 120;

% Speed loop gains. These are engineering design values, not paper-sourced.
p.speed.Kp = 0.48;
p.speed.Ki = 22.0;
p.speed.Kd = 0.0;
p.speed.d_filter_tau = 0.02;

% Current loop gains. Current-loop bandwidth is selected above the speed
% loop; values are exposed here for repeatability and retuning.
p.current.d.Kp = 4.2;
p.current.d.Ki = 850;
p.current.q.Kp = 8.5;
p.current.q.Ki = 850;
p.current.current_ff_gain = 1.0;

% Model-free extremum-seeking optimizer.
p.optimizer.perturbation_amplitude = 0.18;
p.optimizer.omega_d = 2*pi*6.0;
p.optimizer.power_lpf_tau = 0.18;
p.optimizer.gradient_lpf_tau = 0.30;
p.optimizer.alpha = 0.08;
p.optimizer.update_period = p.Ts;
p.optimizer.max_id_bar_step = 1.0e-4;
p.optimizer.id_rate_limit = 5.0;
p.optimizer.resume_ramp_time = 0.05;
p.optimizer.freeze_ramp_time = 0.08;
p.optimizer.voltage_projection_step = 0.02;
p.optimizer.id_bar_initial = 0;

% Steady-state detector and dynamic freeze.
p.steady.omega_error_threshold = 5.0;
p.steady.id_error_threshold = 0.5;
p.steady.iq_error_threshold = 0.5;
p.steady.power_slope_threshold = 2.0;
p.steady.hold_time = 0.10;

% Required experiment scenarios.
p.exp.speed_step.t_step = 0.5;
p.exp.speed_step.omega1_rpm = 1000;
p.exp.speed_step.omega2_rpm = 1600;
p.exp.speed_step.TL = 2.0;
p.exp.constant.omega_rpm = 1200;
p.exp.constant.TL = 2.0;
p.exp.load_step.omega_rpm = 1200;
p.exp.load_step.TL1 = 2.0;
p.exp.load_step.TL2 = 5.0;
p.exp.load_step.t_step = 2.0;
p.exp.perturb.Rs_scale = 1.2;
p.exp.perturb.psi_f_scale = 0.9;
p.exp.perturb.Ld_scale = 1.15;
p.exp.perturb.Lq_scale = 0.85;
p.exp.constraint.omega_rpm = 2200;
p.exp.constraint.TL = 5.0;

p.metric_tail_fraction = 0.2;
p.random_seed = 42;
p.method_order = {'baseline', 'optimization'};
p.method_label.baseline = 'i_d^*=0';
p.method_label.optimization = 'Model-free optimization';
p.version.matlab = 'R2021a';
p.version.simulink = 'R2021a';

dirs = {p.paths.results, p.paths.figures, p.paths.chapter_figures, p.paths.data, p.paths.tables, p.paths.logs, p.paths.models, p.paths.legacy};
for k = 1:numel(dirs)
    if exist(dirs{k}, 'dir') ~= 7
        mkdir(dirs{k});
    end
end

assignin('base', 'p', p);
