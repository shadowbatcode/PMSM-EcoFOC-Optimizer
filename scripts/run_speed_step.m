if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end
scenario.name = 'speed_step';
scenario.Tstop = p.Tstop;
scenario.omega_ref = @(t) (t < p.exp.speed_step.t_step) * (p.exp.speed_step.omega1_rpm * 2*pi/60) + ...
    (t >= p.exp.speed_step.t_step) * (p.exp.speed_step.omega2_rpm * 2*pi/60);
scenario.load_torque = @(t) p.exp.speed_step.TL;
result = run_case_set(scenario, p, 'speed_step');
save(fullfile(p.paths.data, 'speed_step.mat'), 'result', 'scenario', 'p');
