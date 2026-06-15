function [id_projected, info] = Projection_Omega(id_candidate, iq_ref, u_s_est, p)
%PROJECTION_OMEGA Project d-axis reference into current and voltage limits.

info.id_raw = id_candidate;
info.id_after_box = min(max(id_candidate, p.i_d_min), p.i_d_max);
info.box_limited = abs(info.id_after_box - id_candidate) > 1e-12;

id_projected = info.id_after_box;

allowedAbsId = sqrt(max(p.i_s_max^2 - iq_ref^2, 0));
id_current = min(max(id_projected, -allowedAbsId), allowedAbsId);
info.current_limited = abs(id_current - id_projected) > 1e-12;
id_projected = min(max(id_current, p.i_d_min), p.i_d_max);

info.voltage_limited = nargin >= 3 && ~isempty(u_s_est) && u_s_est > p.u_s_max;
if info.voltage_limited
    excess = (u_s_est - p.u_s_max) / max(p.u_s_max, eps);
    id_projected = max(p.i_d_min, id_projected - p.projection_voltage_step * min(10, 1 + 10 * excess));
end

id_projected = min(max(id_projected, p.i_d_min), p.i_d_max);
allowedAbsId = sqrt(max(p.i_s_max^2 - iq_ref^2, 0));
id_projected = min(max(id_projected, -allowedAbsId), allowedAbsId);

info.id_projected = id_projected;
info.constraint_limited = info.box_limited || info.current_limited || info.voltage_limited;
end
