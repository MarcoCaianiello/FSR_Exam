%%
%     %Obstacles
%     circle_radius = 0.5;
%     robot_radius = 0.3;
%     circle1 = circle_obstacle(circle_radius+robot_radius, [5, 5]);
%     circle2 = circle_obstacle(circle_radius+robot_radius, [8, 8]);
%     square1_xy = [1-robot_radius 3.5+robot_radius 3.5+robot_radius 1-robot_radius;
%                   2-robot_radius 2-robot_radius 3+robot_radius 3+robot_radius];
%     square1 = polyshape(square1_xy(1,:), square1_xy(2,:));
% 
%     %Small Obstacles
%     circle1_small = circle_obstacle(circle_radius, [5, 5]);
%     circle2_small = circle_obstacle(circle_radius, [8, 8]);
%     square1_xy_small = [1 3.5 3.5 1 1; 2 2 3 3 2];
% 
% figure;
% hold on; grid on;
% xlabel('x'); ylabel('y');
% xlim([-0.2 10+0.2]); ylim([-0.2 10+0.2]);
% plot([0 10 10 0 0],[0 0 10 10 0],'r--')
% scatter(1,1,[],'g','filled');
% scatter(9,9,[],'r','filled');
% plot(circle1_small.Vertices(:,1),circle1_small.Vertices(:,2),'k','LineWidth',1.5)
% plot(circle2_small.Vertices(:,1),circle2_small.Vertices(:,2),'k','LineWidth',1.5)
% plot(square1_xy_small(1,:), square1_xy_small(2,:),'k','LineWidth',1.5);
% plot(circle1);
% plot(circle2);
% plot(square1);

%%

[x_temp1, y_temp1, theta_temp1, ~, ~] = rectilinear_primitive (0, 0, 0);
[x_temp2, y_temp2, theta_temp2, ~, ~] = circular_primitive (0, 0, 0, 1);
[x_temp3, y_temp3, theta_temp3, ~, ~] = circular_primitive (0, 0, 0, -1);

figure;
hold on; grid on;
xlabel('x'); ylabel('y');
xlim([-0.1 0.1]); ylim([-0.01 0.3]);
plot(y_temp1, x_temp1, 'c', 'LineWidth',2);
plot(y_temp2, x_temp2, 'c', 'LineWidth',2);
plot(y_temp3, x_temp3, 'c', 'LineWidth',2);
scatter(0,0,[],'r','filled');

figure;
subplot(2,1,1);
hold on; grid on;
xlabel('t'); ylabel('rad');
title('Turn left')
plot(linspace(0,0.44,size(theta_temp2,2)), theta_temp2);


subplot(2,1,2);
hold on; grid on;
xlabel('t'); ylabel('rad');
title('Turn right')
plot(linspace(0,0.44,size(theta_temp3,2)), theta_temp3);






