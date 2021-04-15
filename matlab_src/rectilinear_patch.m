%%Rectilinear patch
function [x_patch, y_patch, theta_patch, v_max, w_max] = rectilinear_patch (x_patch, y_patch, xg, yg)

    Ts = 0.01; %10 ms
    v_max   = 0.26;   %m/s
    w_max = 0;
    theta = atan2((yg - y_patch),(xg - x_patch));
    Tv = norm([xg yg] - [x_patch y_patch])/v_max;

    for i =  size(x_patch,2)+1:size(x_patch,2)+round(Tv/Ts)
        [x_patch(i), y_patch(i), theta_patch(i)] = euler_integration (v_max, w_max, x_patch(i-1), y_patch(i-1), theta(1), Ts);
    end

end