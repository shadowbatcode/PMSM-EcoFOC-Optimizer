function result = Exp02_LoadStep(p)
%EXP02_LOADSTEP Load torque step disturbance comparison.

scenario = localBaseScenario('load_step', 'Experiment 2 load torque step', p);
scenario.omega_ref = @(t) p.exp.load_step.omega_ref_rpm * p.rpm_to_rad;
scenario.load_torque = @(t) p.exp.load_step.T_L_initial + ...
    (t >= p.exp.load_step.step_time) * (p.exp.load_step.T_L_final - p.exp.load_step.T_L_initial);
scenario.event_time = p.exp.load_step.step_time;
scenario.seed = 2;

result = localRunMethods(scenario, p);
save(fullfile(p.paths.data, 'exp02_load_step.mat'), 'result');
end

function result = localRunMethods(scenario, p)
for i = 1:numel(p.method_order)
    method = p.method_order{i};
    result.(method) = Simulate_PMSM_FOC(method, scenario, p);
end
result.scenario = scenario;
end

function scenario = localBaseScenario(name, description, p)
scenario.name = name;
scenario.description = description;
scenario.Tsim = p.Tsim;
scenario.Ts = p.Ts;
scenario.perturbed = false;
scenario.nonideal = false;
end
