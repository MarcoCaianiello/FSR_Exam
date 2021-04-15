%% Euler Integration
function [x, y, theta] = euler_integration (v, w, x_old, y_old, theta_old, Ts)

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


x = x_old + v*Ts*cos(theta_old);
y = y_old + v*Ts*sin(theta_old);
theta = theta_old + w*Ts;

end