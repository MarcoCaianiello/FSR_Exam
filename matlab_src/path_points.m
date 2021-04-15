function [x_path, y_path, theta_path, velocities] = path_points (path)

    x_path=[];
    y_path=[];
    theta_path=[];
    velocities = [];
    
    for i=1:(size(path,1)-2)
            
        theta_diff = path(i+1,3) - path(i,3);
        if theta_diff == 0 
            [x_temp, y_temp, theta_temp, v_temp, w_temp] = rectilinear_primitive (path(i,1), path(i,2), path(i,3));
        elseif theta_diff > 0 
            [x_temp, y_temp, theta_temp, v_temp, w_temp] = circular_primitive (path(i,1), path(i,2), path(i,3), 1);
        else
            [x_temp, y_temp, theta_temp, v_temp, w_temp] = circular_primitive (path(i,1), path(i,2), path(i,3), -1);
        end
        
        if (i==size(path,1)-2)
            k = dsearchn([x_temp' y_temp'], [path(end, 1) path(end, 2)]);
            x_temp = x_temp(1,1:k);
            y_temp = y_temp(1,1:k);
            theta_temp = theta_temp(1,1:k);
        end
            
        if isempty(x_path)
            x_path = x_temp';
            y_path = y_temp';
            theta_path = theta_temp';
        else
            x_path = [x_path; x_temp'];
            y_path = [y_path; y_temp'];
            theta_path= [theta_path; theta_temp'];
        end
        v_temp = ones(size(x_temp,2),1).*v_temp;
        w_temp = ones(size(x_temp,2),1).*w_temp;
        velocities = [velocities; [v_temp w_temp]];
    end
    
    [x_patch, y_patch, theta_patch, v_temp, w_temp] = rectilinear_patch(x_temp(end), y_temp(end), path(end, 1), path(end, 2));
    x_path = [x_path; x_patch'];
    y_path = [y_path; y_patch'];
    theta_path= [theta_path; theta_patch'];
    v_temp = ones(size(x_patch,2),1).*v_temp;
    w_temp = ones(size(x_patch,2),1).*w_temp;
    velocities = [velocities; [v_temp w_temp]];
    velocities(end,:) = zeros(1,2);
end