% analyzing ETHZ data
%% Analsyis for Velocity
clc;
close all;
figure();
title('Velocity Tracking via EKF and UKF');
xlabel('Time [\mu s]')
ylabel('V_x [m s^{-1}]')
hold on
plot(RawAccel(26900:27000,1), x_ukf(3, 26900:27000),'-x') % UKF Estimation
plot(RawAccel(26900:27000,1), x(3,26900:27000),'-o') % EKF Estimation
plot(RawAccel(26900:27000,1), Y(26900:27000,3),'-s') % real data
hold off
saveas(gcf,'velocity_tracking.png')

clc
ukf_check_vx = (x_ukf(3,1:27000)' - Y(1:27000,3));
ukf_RMS_vx = mean(norm(ukf_check_vx,'fro'));
ekf_check_vx = (x(3,1:27000)' - Y(1:27000,3));
ekf_RMS_vx = mean(norm(ekf_check_vx,'fro'));

disp(ukf_RMS_vx)
disp(ekf_RMS_vx)

%% X Position Analysis
clc;
close all;
figure();
hold on
title('X Position Tracking versus Time via EKF and UKF');
xlabel('Time [\mu s]')
ylabel('x [m]')
plot(RawAccel(26900:26950, 1),x_ukf(1, 26900:26950),'-x') % UKF Estimation
plot(RawAccel(26900:26950, 1),x(1,26900:26950),'-o') % EKF Estimation
plot(RawAccel(26900:26950, 1),x_input(26900:26950,1)-x_input(1,1),'-s') % real data
hold off

ukf_check_y = (x_ukf(2,1:27000)' - (y_input(1:27000,1)-y_input(1,1)));
ukf_RMS_pos_y =(norm(ukf_check_y,'fro')/sqrt(27000));
ekf_check_y = (x(2,1:27000)' - (y_input(1:27000,1)-y_input(1,1)));
ekf_RMS_pos_y =(norm(ekf_check_y,'fro')/sqrt(27000));
ekf_RMS_pos_y_vec = ekf_check_y.^(0.5)
ukf_RMS_pos_y_vec = ukf_check_y.^0.5
saveas(gcf,'velocity_tracking_with_time.png')
disp(ukf_RMS_pos_y)
disp(ekf_RMS_pos_y)
%%
close all;
figure()
XMIN = 7E+6
XMAX = 1E+7
YMIN = -1
YMAX = 550
hold on
scatter(RawAccel(1:27000,1),ekf_RMS_pos_y_vec)
scatter(RawAccel(1:27000,1),ukf_RMS_pos_y_vec)
legend('EKF','UKF');
axis([XMIN XMAX YMIN YMAX])
title('MMSE for EKF and UKF as Compared with the Input');
xlabel('Time [s]')
ylabel('X Position [m]');
hold off
saveas(gcf,'convergence_error_nstuff.png')

%%
ukf_check_ygt = (x_ukf(2,1:27000)' - (y_gt(1:27000,1)-y_gt(1,1)));
ukf_RMS_pos_ygt =(norm(ukf_check_ygt,'fro')/sqrt(27000));
ekf_check_ygt = (x(2,1:27000)' - (y_gt(1:27000,1)-y_gt(1,1)));
ekf_RMS_pos_ygt =(norm(ekf_check_ygt,'fro')/sqrt(27000));
ekf_RMS_pos_y_vecgt = ekf_check_ygt.^(0.5)
ukf_RMS_pos_y_vecgt = ukf_check_ygt.^0.5
figure()
% XMIN = 7E+6
% XMAX = 1E+7
% YMIN = -1
% YMAX = 550
hold on
scatter(RawAccel(1:27000,1),ekf_RMS_pos_y_vec)
scatter(RawAccel(1:27000,1),ukf_RMS_pos_y_vec)
legend('EKF','UKF');
% axis([XMIN XMAX YMIN YMAX])
title('MMSE for EKF and UKF as Compared with the Input');
xlabel('Time [s]')
ylabel('X Position [m]');
hold off
saveas(gcf,'convergence_error_nstuff.png')
%% GPS

close all;
figure();

hold on
% plot(x_ukf(1, 26500:27000),'-x') % UKF Estimation
% plot(x(1,26500:27000),'-o') % EKF Estimation
scatter(output(:,1)-output(1,1),output(:,2)-output(1,2),'x','b')
scatter(x_input(1:27000,1)-x_input(1,1),y_input(1:27000)-y_input(1,1),'r') % real data
hold off
%%
close all;
figure();
hold on
XMIN = 4.5E+8
XMAX = 5.3E+8
YMIN = 25
YMAX = 42
axis([XMIN XMAX YMIN YMAX])
title('Input Position (X) Based on Sensory Measurements for EKF and UKF');
xlabel('Time [\mu s]')
ylabel('X [m]')
scatter(RawAccel(1:27000,1),x_input(1:27000,1)-x_input(1,1),'r') % real data
scatter(OnboardGPS(:,1), output(:,1)-output(1,1),'g');
hold off
saveas(gcf,'sensory_x.png')
figure();
hold on
XMIN = 4.8E+8
XMAX = 4.85E+8
YMIN = 26
YMAX = 30
axis([XMIN XMAX YMIN YMAX])
title('Input Position Based on Sensory Measurements for EKF and UKF');
xlabel('Time [\mu s]')
ylabel('Y [m]')
scatter(RawAccel(1:27000,1),y_input(1:27000,1)-y_input(1,1),'r') % real data
scatter(OnboardGPS(:,1), output(:,2)-output(1,2),'g');
saveas(gcf,'sensory_y.png')
hold off