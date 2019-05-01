% analyzing ETHZ data
%% Analsyis for Velocity
clc;
close all;
figure();
hold on
plot(x_ukf(3, 26500:27000),'-x') % UKF Estimation
plot(x(3,26500:27000),'-o') % EKF Estimation
plot(Y(26500:27000,3),'-s') % real data
hold off
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
plot(RawAccel(26500:27000, 1),x_ukf(1, 26500:27000),'-x') % UKF Estimation
plot(RawAccel(26500:27000, 1),x(1,26500:27000),'-o') % EKF Estimation
plot(RawAccel(26500:27000, 1),x_input(26500:27000,1)-x_input(1,1),'-s') % real data
hold off

ukf_check_x = (x_ukf(1,1:27000)' - (x_input(1:27000,1)-x_input(1,1)));
ukf_RMS_pos_x =(norm(ukf_check_x,'fro')/sqrt(27000));
ekf_check_x = (x(1,1:27000)' - (x_input(1:27000,1)-x_input(1,1)));
ekf_RMS_pos_x =(norm(ekf_check_x,'fro')/sqrt(27000));

disp(ukf_RMS_pos_x)
disp(ekf_RMS_pos_x)
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
scatter(RawAccel(1:27000,1),x_input(1:27000,1)-x_input(1,1),'r') % real data
scatter(OnboardGPS(:,1), output(:,1)-output(1,1),'g');
hold off
figure();
hold on
scatter(RawAccel(1:27000,1),y_input(1:27000,1)-y_input(1,1),'r') % real data
scatter(OnboardGPS(:,1), output(:,2)-output(1,2),'g');
hold off