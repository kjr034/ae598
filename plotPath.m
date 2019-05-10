% clear
% close
clc
% load the data into matlab 
% run('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\loadGroundTruthAGL.m');
run('/home/johnkan2/ae598/final_proj/ae598/AGZ_subset/loadGroundTruthAGL.m');
% load('onboardgps.mat')
% plot ground truth positions
%% Plot the stuff
clc;
close all;
% pos_vec = lla2flat( [Y(:,2:-1:1), zeros(N,1)], [0 90], 0, -1000, 'WGS84' );
% gps_vec = ll2utm([Y(:,2)])
% gps_new = lla2flat([x(1,:)' x(2,:)' zeros(27000,1)], [0 32], -5, 100)
% clear x_orig
%%
clc
close all;
figure();
hold on
title('Normalized Position Tracking via EKF and UKF');
xlabel('X [m]')
ylabel('Y [m]')
scatter(x_input(10000:12048,1)-x_input(1,1),y_input(10000:12048,1)-y_input(1,1),'x');
% scatter(x(1,1:27000), x(2,1:27000),'x','r');
scatter(x_ekf(1,10000:12048), x_ekf(2,10000:12048),'o','b');
% scatter(x_gt-x_gt(1),y_gt-y_gt(1));
% scatter(x_ukf(1,1:25000),x_ukf(2,1:25000),'.');
% scatter(x_gps-x_gps(1), y_gps-y_gps(1),'x','g');
legend('EKF','UKF','GPS Measurement');
% scatter(gps_vec(:,1)-x_gt(1), gps_vec(:,2)-x_gt(2),'x')

hold off

saveas(gcf,'path_plot_final.png')
%%
clc
close all;
figure();
XMIN = 90
XMAX = 110
YMIN = 40
YMAX = 62
hold on
title('Normalized Position Tracking via EKF and UKF');
xlabel('X [m]')
ylabel('Y [m]')
axis([XMIN XMAX YMIN YMAX])
% plot(x_input(:,1)-x_input(1,1),y_input(:,1)-y_input(1,1),'-o');
scatter(x(1,1:27000), x(2,1:27000),'o','r');
% scatter(x_gt-x_gt(1),y_gt-y_gt(1));
scatter(x_ukf(1,1:27000),x_ukf(2,1:27000),'.');
scatter(x_gps-x_gps(1), y_gps-y_gps(1),'x','g');
legend('EKF','UKF','GPS Measurement');
% scatter(gps_vec(:,1)-x_gt(1), gps_vec(:,2)-x_gt(2),'x')

hold off

saveas(gcf,'path_plot_final_zoom.png')


%%

figure();
hold on

plot(x_ukf(3, 26900:27000),'-x') % UKF Estimation
plot(x(3,26900:27000),'-o') % EKF Estimation
plot(Y(26900:27000,3),'-s') % real data
hold off
% plot3(x_gt, y_gt, zeros(2708,1), '.')
% grid on
% hold on
% % plot gps positions
% plot3(x_gps, y_gps, zeros(2708,1), 'or')
% axis equal
% axis vis3d
%%

%%

a = mahal([x_gt(1),y_gt(1)],[x(1,10:27000); x(2,10:27000)]')
b = mahal([x_gt(1),y_gt(1)],[x_gt, y_gt])
c = mahal([x_gt(1),y_gt(1)],[x_gps,y_gps])
d = mahal([x_gt(1),y_gt(1)],[x_ukf(1,10:27000); x_ukf(2,10:27000)]')