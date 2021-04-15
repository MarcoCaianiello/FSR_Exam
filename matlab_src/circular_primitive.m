%%Circular primitive
function [x, y, theta, v_max, w_max] = circular_primitive (x, y, theta, direction)

    Tw = 0.44;
    Ts = 0.01; %10 ms
    v_max   = 0.26;   %m/s
    w_max   = 1.86;   %rad/s
    
    if direction >= 0
        direction = 1;
    else
        direction = -1;
    end

    for i =  size(x,2)+1:size(x,2)+round(Tw/Ts)
    [x(i), y(i), theta(i)] = euler_integration (v_max, direction*w_max, x(i-1), y(i-1), theta(i-1), Ts);
    end

end