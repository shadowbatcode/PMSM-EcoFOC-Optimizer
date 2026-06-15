function result = Exp06_Constraints(p)
%EXP06_CONSTRAINTS High-speed/high-load current and voltage constraint case.

scenario = localBaseScenario('constraints', 'Experiment 6 current and voltage constraints', p);
scenario.omega_ref = @(t) p.exp.constraint.omega_ref_rpm * p.rpm_to_rad;
scenario.load_torque = @(t) p.exp.constraint.T_L;
scenario.seed = 8;

result = localRunMethods(scenario, p);
save(fullfile(p.paths.data, 'exp06_constraints.mat'), 'result');
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
