function result = Exp01_ConstantLoad(p)
%EXP01_CONSTANTLOAD Constant speed and constant load comparison.

scenario = baseScenario('constant_load', 'Experiment 1 constant speed and load', p);
scenario.omega_ref = @(t) p.exp.constant.omega_ref_rpm * p.rpm_to_rad;
scenario.load_torque = @(t) p.exp.constant.T_L;
scenario.seed = 1;

result = runMethods(scenario, p);
save(fullfile(p.paths.data, 'exp01_constant_load.mat'), 'result');
end

function result = runMethods(scenario, p)
for i = 1:numel(p.method_order)
    method = p.method_order{i};
    result.(method) = Simulate_PMSM_FOC(method, scenario, p);
end
result.scenario = scenario;
end

function scenario = baseScenario(name, description, p)
scenario.name = name;
scenario.description = description;
scenario.Tsim = p.Tsim;
scenario.Ts = p.Ts;
scenario.perturbed = false;
scenario.nonideal = false;
end
