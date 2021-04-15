function [] = Astar_plot (height, width, x_s, y_s, x_g, y_g, goal_range, robot_radius, path, x_path, y_path, theta_path)
    
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
    
    %Goal range
    goal_circle = circle_obstacle(goal_range, [x_g, y_g]);
    
    figure; 
    hold on; grid on;
    xlabel('x'); ylabel('y');
    xlim([-0.2 width+0.2]); ylim([-0.2 height+0.2]);
    plot([0 width width 0 0],[0 0 height height 0],'r--')
    plot(x_path, y_path, 'y.');
    scatter(path(:,1),path(:,2),4,'b','filled');
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