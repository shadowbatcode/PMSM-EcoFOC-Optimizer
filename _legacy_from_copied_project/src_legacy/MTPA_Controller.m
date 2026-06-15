function id_ref = MTPA_Controller(iq_ref, motor)
%MTPA_CONTROLLER Nominal IPMSM MTPA d-axis current reference.

deltaL = motor.L_q - motor.L_d;
if abs(deltaL) < 1e-12
    id_ref = 0;
else
    id_ref = (motor.psi_f - sqrt(motor.psi_f^2 + 8 * deltaL^2 * iq_ref^2)) / (4 * deltaL);
end

id_ref = min(max(id_ref, motor.i_d_min), motor.i_d_max);
allowed = sqrt(max(motor.i_s_max^2 - iq_ref^2, 0));
id_ref = min(max(id_ref, -allowed), allowed);
id_ref = min(max(id_ref, motor.i_d_min), motor.i_d_max);
end
