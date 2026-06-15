function result = Exp03_SpeedStep(p)
%EXP03_SPEEDSTEP Speed reference step comparison.

scenario = localBaseScenario('speed_step', 'Experiment 3 speed reference step', p);
scenario.omega_ref = @(t) (p.exp.speed_step.omega_ref_initial_rpm + ...
    (t >= p.exp.speed_step.step_time) * ...
    (p.exp.speed_step.omega_ref_final_rpm - p.exp.speed_step.omega_ref_initial_rpm)) * p.rpm_to_rad;
scenario.load_torque = @(t) p.exp.speed_step.T_L;
scenario.event_time = p.exp.speed_step.step_time;
scenario.seed = 3;

result = localRunMethods(scenario, p);
save(fullfile(p.paths.data, 'exp03_speed_step.mat'), 'result');
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
