function pmsm_foc_sfunc(block)
%PMSM_FOC_SFUNC Level-2 MATLAB S-Function wrapper for the PMSM FOC engine.

setup(block);
end

function setup(block)
block.NumDialogPrms = 3; % parameter struct, method, scenario
block.NumInputPorts = 0;
block.NumOutputPorts = 1;
block.SetPreCompOutPortInfoToDynamic;
block.OutputPort(1).Dimensions = numel(pmsm_foc_output_names());
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
block.SampleTimes = [block.DialogPrm(1).Data.Ts 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup', @post_propagation_setup);
block.RegBlockMethod('InitializeConditions', @initialize_conditions);
block.RegBlockMethod('Outputs', @outputs);
end

function post_propagation_setup(block)
block.NumDworks = 3;

block.Dwork(1).Name = 'engine_state';
block.Dwork(1).Dimensions = 18;
block.Dwork(1).DatatypeID = 0;
block.Dwork(1).Complexity = 'Real';
block.Dwork(1).UsedAsDiscState = true;

block.Dwork(2).Name = 'engine_output';
block.Dwork(2).Dimensions = numel(pmsm_foc_output_names());
block.Dwork(2).DatatypeID = 0;
block.Dwork(2).Complexity = 'Real';
block.Dwork(2).UsedAsDiscState = true;

block.Dwork(3).Name = 'last_time';
block.Dwork(3).Dimensions = 1;
block.Dwork(3).DatatypeID = 0;
block.Dwork(3).Complexity = 'Real';
block.Dwork(3).UsedAsDiscState = true;
end

function initialize_conditions(block)
p = block.DialogPrm(1).Data;
block.Dwork(1).Data = state_to_vector(initial_state(p));
block.Dwork(2).Data = zeros(numel(pmsm_foc_output_names()), 1);
block.Dwork(3).Data = -inf;
end

function outputs(block)
p = block.DialogPrm(1).Data;
method = block.DialogPrm(2).Data;
scenario = block.DialogPrm(3).Data;

x = vector_to_state(block.Dwork(1).Data);
if block.CurrentTime > block.Dwork(3).Data + 0.5 * p.Ts
    omega_ref = scenario.omega_ref(block.CurrentTime);
    T_L = scenario.load_torque(block.CurrentTime);
    [x, y] = pmsm_foc_step(x, p, omega_ref, T_L, method, p.Ts);
    block.Dwork(1).Data = state_to_vector(x);
    block.Dwork(2).Data = [y.omega_m; y.omega_rpm; y.id; y.iq; y.id_ref; y.iq_ref; ...
        y.ud; y.uq; y.us; y.is; y.Te; y.Pin; y.Pin_lpf; y.Pout; y.eta; ...
        y.speed_error; y.perturbation; y.P_ac; y.demod_signal; y.g_hat; ...
        y.id_bar; y.id_bar_before; y.steady_flag; y.optimizer_enable; ...
        y.freeze_state; y.projection_active; y.current_saturated; ...
        y.voltage_saturated; y.dPin_dt];
    block.Dwork(3).Data = block.CurrentTime;
end

block.OutputPort(1).Data = block.Dwork(2).Data;
end

function x = initial_state(p)
x.t = 0;
x.id = 0;
x.iq = 0;
x.omega_m = 0;
x.speed_int = 0;
x.id_int = 0;
x.iq_int = 0;
x.Pin = 0;
x.Pin_prev = 0;
x.Pin_lpf = 0;
x.g_hat_lpf = 0;
x.id_bar = p.optimizer.id_bar_initial;
x.u_s = 0;
x.id_ref_prev = 0;
x.steady_count = 0;
x.perturbation_ramp = 0;
x.current_saturated = false;
x.voltage_saturated = false;
end

function v = state_to_vector(x)
v = [x.t; x.id; x.iq; x.omega_m; x.speed_int; x.id_int; x.iq_int; ...
    x.Pin; x.Pin_prev; x.Pin_lpf; x.g_hat_lpf; x.id_bar; x.u_s; ...
    x.id_ref_prev; x.steady_count; x.perturbation_ramp; ...
    double(x.current_saturated); double(x.voltage_saturated)];
end

function x = vector_to_state(v)
x.t = v(1);
x.id = v(2);
x.iq = v(3);
x.omega_m = v(4);
x.speed_int = v(5);
x.id_int = v(6);
x.iq_int = v(7);
x.Pin = v(8);
x.Pin_prev = v(9);
x.Pin_lpf = v(10);
x.g_hat_lpf = v(11);
x.id_bar = v(12);
x.u_s = v(13);
x.id_ref_prev = v(14);
x.steady_count = v(15);
x.perturbation_ramp = v(16);
x.current_saturated = v(17) > 0.5;
x.voltage_saturated = v(18) > 0.5;
end
