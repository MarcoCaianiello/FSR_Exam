%% MAIN CODE

close all; clear all; clc;

%Work area
height = 10;
width = 10;

%Start
x_s = 1;
y_s = 1;
theta_s = 0;

%Goal
x_g = 9;
y_g = 9;
theta_g = 0;
goal_range = 0.1;

max_iteration = 10000;
robot_radius = 0.3;
rrt_plot_flag = 1; %set to zero for disabling plot

%% RRT 
[x_tree, y_tree, theta_tree, vert, A] = RRT_function (height, width, x_s, y_s, theta_s, x_g, y_g, goal_range, max_iteration, robot_radius, rrt_plot_flag);

%% A* 
% path = Astar_function(x_s, y_s, theta_s, x_g, y_g, vert, A);
path = Astar_function(x_s, y_s, theta_s, x_g, y_g, vert, A);
[x_path, y_path, theta_path, velocities] = path_points(path);
%%
Astar_plot(height, width, x_s, y_s, x_g, y_g, goal_range, robot_radius, path, x_path, y_path, theta_path);

%%
% fprintf(fopen('x_path.txt','w'),'%.4f\n',x_path);
% fprintf(fopen('y_path.txt','w'),'%.4f\n',y_path);
% fprintf(fopen('theta_path.txt','w'),'%.4f\n',theta_path);
% fprintf(fopen('lin_vel.txt','w'),'%.4f\n',velocities(:,1));
% fprintf(fopen('ang_vel.txt','w'),'%.4f\n',velocities(:,2));

%%
% ang_error = fscanf(fopen('ang_error.txt','r'),'%f');
% pos_error = fscanf(fopen('pos_error.txt','r'),'%f');
% lin_vel_real = fscanf(fopen('lin_vel_real.txt','r'),'%f');
% ang_vel_real = fscanf(fopen('ang_vel_real.txt','r'),'%f');
%%
% plot_risultati(pos_error, ang_error, lin_vel_real, ang_vel_real);






