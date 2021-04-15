function [x_tree, y_tree, theta_tree, vert, A] = RRT_function (height, width, x_s, y_s, theta_s, x_g, y_g, goal_range, max_iteration, robot_radius, plot_flag)

    center = [width/2, height/2];
    goal_circle = circle_obstacle(goal_range, [x_g, y_g]);
    
    %Tree
    x_tree(1) = x_s;
    y_tree(1) = y_s;
    theta_tree(1) = theta_s;

    find_goal = 0;
    iteration = 1;

    %Tree initialization
    vert = [x_s y_s theta_s norm([x_g y_g]-[x_s y_s])];
    vert_count = 1;
    index = zeros(iteration,1);

    %Graph matrix
    A = zeros(1,1);

    %Obstacles
    circle_radius = 0.5;
    circle1 = circle_obstacle(circle_radius+robot_radius, [7, 8]);
    circle2 = circle_obstacle(circle_radius+robot_radius, [8, 4.5]);
    square1_xy = [3-robot_radius 5.5+robot_radius 5.5+robot_radius 3-robot_radius;
                  2-robot_radius 2-robot_radius 3+robot_radius 3+robot_radius];
    square1 = polyshape(square1_xy(1,:), square1_xy(2,:));

    %Small Obstacles
    circle1_small = circle_obstacle(circle_radius, [7, 8]);
    circle2_small = circle_obstacle(circle_radius, [8, 4.5]);
    square1_xy_small = [3 5.5 5.5 3 3; 2 2 3 3 2];

    %Reinitialize the random number generator
    %with a seed based on the current time
    rng('shuffle');
    
    tic;
    while (iteration < max_iteration) && (~find_goal)
        %Generate random position
        q_rand(1) =  height*rand();
        q_rand(2) =  width*rand();

        %Find the nearest point
        index(iteration) = dsearchn(vert(:,1:2), q_rand);

        %Draw the primitive
        if rem(round(rand()),2) == 0
            [x_temp, y_temp, theta_temp, ~, ~] = rectilinear_primitive (vert(index(iteration),1), vert(index(iteration),2), vert(index(iteration),3));
        else
            if rem(round(rand()),2) == 0
                [x_temp, y_temp, theta_temp, ~, ~] = circular_primitive (vert(index(iteration),1), vert(index(iteration),2), vert(index(iteration),3), 1);
            else
                [x_temp, y_temp, theta_temp, ~, ~] = circular_primitive (vert(index(iteration),1), vert(index(iteration),2), vert(index(iteration),3), -1);
            end
        end

        %Check validity
        bound_condition = (prod(x_temp<height) && prod(y_temp<height) && prod(x_temp>0) && prod(y_temp>0));

        [in, on] = inpolygon(x_temp,y_temp,circle1.Vertices(:,1),circle1.Vertices(:,2));
        circle1_condition = (numel(x_temp(in)) == 0) && (numel(y_temp(in)) == 0) && (numel(x_temp(on)) == 0) && (numel(y_temp(on)) == 0);

        [in, on] = inpolygon(x_temp,y_temp,square1.Vertices(:,1),square1.Vertices(:,2));
        square1_condition = (numel(x_temp(in)) == 0) && (numel(y_temp(in)) == 0) && (numel(x_temp(on)) == 0) && (numel(y_temp(on)) == 0);

        [in, on] = inpolygon(x_temp,y_temp,circle2.Vertices(:,1),circle2.Vertices(:,2));
        circle2_condition = (numel(x_temp(in)) == 0) && (numel(y_temp(in)) == 0) && (numel(x_temp(on)) == 0) && (numel(y_temp(on)) == 0);
        
        duplicate_condition = 0;
        i = 1;
        while ~duplicate_condition && i<=size(vert(:,1),1)
            dist = norm(vert(i,1:2)-[x_temp(end) y_temp(end)]);
            if dist<0.001
                duplicate_condition=1;
            end
            i = i+1;
        end
        
        %Update Tree
        if (bound_condition && circle1_condition && square1_condition && circle2_condition && ~duplicate_condition)

            [in, on] = inpolygon(x_temp,y_temp,goal_circle.Vertices(:,1),goal_circle.Vertices(:,2));
            if ((numel(x_temp(in)) > 0) && (numel(y_temp(in)) > 0)) || ((numel(x_temp(on)) > 0) && (numel(y_temp(on)) > 0))
                find_goal=1;
                k = dsearchn([x_temp' y_temp'], [x_g y_g]);
                x_temp = x_temp(1,1:k);
                y_temp = y_temp(1,1:k);
                theta_temp = theta_temp(1,1:k);
                [x_patch, y_patch, theta_patch, ~, ~] = rectilinear_patch(x_temp(end), y_temp(end), x_g, y_g);
            end


            %Add a primitive to the Tree
            x_tree = [x_tree x_temp];
            y_tree = [y_tree y_temp];
            theta_tree = [theta_tree theta_temp];
            

            vert_count = vert_count + 1;
            cost = Astar_FN(vert(index(iteration),1:2), [x_temp(end) y_temp(end)], [x_g y_g], vert(index(iteration),4));
            vert(vert_count,:) = [x_temp(end), y_temp(end), theta_temp(end), cost];

            %Build graph matrix
            A(vert_count,:)=0;
            A(:,vert_count)=0;
            A(index(iteration), vert_count)=1;

        end
        
        if find_goal
            x_tree = [x_tree x_patch];
            y_tree = [y_tree y_patch];
            theta_tree = [theta_tree theta_patch];
            
            vert_count = vert_count + 1;
            vert(vert_count,:) = [x_g, y_g, 0, 0];
            
            %Build graph matrix
            A(vert_count,:)=0;
            A(:,vert_count)=0;
            A(vert_count-1, vert_count)=1;
            
        end
        
        iteration = iteration+1;
    end
    toc;
    
    %% Plot Section
    if plot_flag
        figure;
        hold on; grid on;
        xlabel('x'); ylabel('y');
        xlim([-0.2 width+0.2]); ylim([-0.2 height+0.2]);
        plot([0 width width 0 0],[0 0 height height 0],'r--')
        plot(x_tree, y_tree, 'y.');
        plot(x_patch, y_patch, 'r.');
        scatter(vert(:,1),vert(:,2),4,'b','filled');
        scatter(x_s,y_s,[],'g','filled');
        scatter(x_g,y_g,[],'r','filled');
        plot(goal_circle.Vertices(:,1),goal_circle.Vertices(:,2),'r')
        plot(circle1_small.Vertices(:,1),circle1_small.Vertices(:,2),'k','LineWidth',1.5)
        plot(circle2_small.Vertices(:,1),circle2_small.Vertices(:,2),'k','LineWidth',1.5)
        plot(square1_xy_small(1,:), square1_xy_small(2,:),'k','LineWidth',1.5);
        plot(circle1);
        plot(circle2);
        plot(square1);
    end


end