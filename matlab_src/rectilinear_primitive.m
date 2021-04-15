%%Rectilinear primitive
function [x, y, theta, v_max, w_max] = rectilinear_primitive (x, y, theta)

    Tv = 1;
    Ts = 0.01; %10 ms
    v_max   = 0.26;   %m/s
    w_max = 0;

    for i =  size(x,2)+1:size(x,2)+round(Tv/Ts)
        [x(i), y(i), theta(i)] = euler_integration (v_max, w_max, x(i-1), y(i-1), theta(i-1), Ts);
    end

end