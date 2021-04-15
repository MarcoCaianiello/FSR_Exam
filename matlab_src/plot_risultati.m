%%
function [] = plot_risultati (pos_error, ang_error, lin_vel_real, ang_vel_real)
pos_error(1,:)=[];
figure;
hold on; grid on;
xlabel('samples'); ylabel('position error');
xlim([0 size(pos_error,1)]);
plot(pos_error);
plot([1 size(pos_error,1)], [mean(pos_error) mean(pos_error)],'r-')

% ang_error(find(ang_error<0)) = ang_error(find(ang_error<0))+2*pi;
figure;
hold on; grid on;
xlabel('samples'); ylabel('angle error');
xlim([0 size(ang_error,1)]);
plot(ang_error);

figure;
hold on; grid on;
xlabel('samples'); ylabel('Linear Velocities');
xlim([0 size(lin_vel_real,1)]); ylim([-0.3 0.3]);
plot(lin_vel_real);

figure;
hold on; grid on;
xlabel('samples'); ylabel('Angular Velocities');
xlim([0 size(ang_vel_real,1)]); ylim([-1.72 1.72]);
plot(ang_vel_real);

end