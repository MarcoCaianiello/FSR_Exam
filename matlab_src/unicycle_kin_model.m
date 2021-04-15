%% Kinematic model
function q_dot = unicycle_kin_model (v, w, theta)
v_max = 0.26;   %m/s
w_max = 1.86;   %rad/s

if v > v_max
    v = v_max;
elseif v < -v_max
    v = -v_max;
end

if w > w_max
    w = w_max;
elseif w < -w_max
    w = -w_max;
end

G = [cos(theta) sin(theta) 0; 0 0 1;];
q_dot = G * [v; w];
end