function p = Parameter_Set()
%PARAMETER_SET Central parameter file for the PMSM FOC experiments.
% The motor parameters below are typical IPMSM values and can be replaced
% by measured prototype parameters before using the results as hardware data.

srcDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(srcDir);

p.project_root = projectRoot;
p.paths.src = srcDir;
p.paths.experiments = fullfile(projectRoot, 'experiments');
p.paths.results = fullfile(projectRoot, 'results');
p.paths.data = fullfile(projectRoot, 'results', 'data');
p.paths.figures = fullfile(projectRoot, 'results', 'figures');
p.paths.tables = fullfile(projectRoot, 'results', 'tables');
p.paths.simulink = fullfile(projectRoot, 'simulink');
p.paths.simulink_model = fullfile(p.paths.simulink, 'PMSM_FOC_Model.slx');

% Motor and inverter nominal parameters.
p.R_s = 0.80;             % Ohm
p.L_d = 3.5e-3;           % H
p.L_q = 8.5e-3;           % H
p.psi_f = 0.105;          % Wb
p.p = 4;                  % pole pairs
p.J_m = 2.0e-3;           % kg*m^2
p.B = 5.0e-4;             % N*m*s/rad
p.U_dc = 300;             % V
p.u_s_max = p.U_dc / sqrt(3);

% Current and reference limits.
p.i_s_max = 25.0;         % A
p.i_d_min = -18.0;        % A
p.i_d_max = 4.0;          % A
p.i_q_max = 24.0;         % A

% Simulation setup.
p.Ts = 1.0e-4;            % s
p.Tsim = 4.0;             % s
p.rpm_to_rad = 2*pi/60;
p.rad_to_rpm = 60/(2*pi);
p.random_seed = 42;

% Current-loop PI gains.
p.K_pd = 4.2;
p.K_id = 850;
p.K_pq = 8.5;
p.K_iq = 850;

% Speed-loop PID gains. K_dw defaults to zero to avoid noise amplification.
p.K_pw = 0.48;
p.K_iw = 22.0;
p.K_dw = 0.0;

% Model-free optimizer parameters.
p.alpha = 0.08;
p.a = 0.18;                       % A perturbation amplitude
p.omega_p = 2*pi*6.0;             % rad/s perturbation frequency
p.lpf_tau = 0.18;                 % s
p.demod_tau = 0.30;               % s
p.opt_smooth = 0.004;
p.g_hat_limit = 20.0;             % W/A, protects slow optimizer from demodulation ripple
p.id_bar_step_max = 1.0e-4;       % A/sample maximum outer-loop update
p.power_backtrack_margin = 0.05;  % W
p.power_backtrack_gain = 0.05;
p.best_hold_time = 0.60;          % s before power backtracking is enabled
p.projection_voltage_step = 0.02; % A/sample heuristic under voltage limit

% Steady-state detector parameters.
p.epsilon_w = 5.0;                % rad/s
p.epsilon_i = 0.50;               % A
p.epsilon_P = 2.0;                % W
p.N_s = 1000;                     % samples

% Metric and plotting defaults.
p.metric_tail_fraction = 0.20;
p.settling_tol_2pct = 0.02;
p.settling_tol_5pct = 0.05;
p.methods = {'id0', 'mtpa', 'mfo'};
p.method_order = {'id0', 'mtpa', 'mfo'};
p.method_label.id0 = 'i_d^*=0';
p.method_label.mtpa = 'MTPA';
p.method_label.mfo = 'Model-free';

% Experiment settings.
p.exp.constant.omega_ref_rpm = 1000;
p.exp.constant.T_L = 2.0;
p.exp.load_step.omega_ref_rpm = 1000;
p.exp.load_step.T_L_initial = 2.0;
p.exp.load_step.T_L_final = 5.0;
p.exp.load_step.step_time = 1.5;
p.exp.speed_step.omega_ref_initial_rpm = 1000;
p.exp.speed_step.omega_ref_final_rpm = 2000;
p.exp.speed_step.T_L = 2.0;
p.exp.speed_step.step_time = 1.5;
p.exp.constraint.omega_ref_rpm = 2500;
p.exp.constraint.T_L = 5.0;

% Non-ideal engineering factors for experiment 5.
p.noise.current_std = 0.06;       % A
p.noise.speed_std = 0.80;         % rad/s
p.noise.delay_time = 2.0e-3;      % s first-order measurement lag
p.noise.udc_ripple_amp = 0.015;   % relative
p.noise.udc_ripple_freq = 100;    % Hz

% Parameter perturbation for experiment 4.
p.perturb.R_s_scale = 1.30;
p.perturb.psi_f_scale = 0.90;
p.perturb.L_d_scale = 1.15;
p.perturb.L_q_scale = 0.85;
end
