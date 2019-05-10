%% EKF
close all;
num_points_vec = [100, 1000, 10000, 15000, 20000, 25000];
ekf_time_elapsed =   [  0.055543, 0.546664, 4.619367, 6.915015, 9.091876, 11.214172;
                    0.050048, 0.464550, 4.456990, 6.849361, 8.968956, 11.337758;
                    0.050605, 0.503496, 4.548662, 6.799907, 8.923555, 11.326292 ];
ekf_mean_time_vec = mean(ekf_time_elapsed)
figure()
scatter(num_points_vec, ekf_mean_time_vec)
title('EKF: Number of Points versus Mean Time');
xlabel('Number of Points');
ylabel('Time Elapsed (s)');
saveas(gcf,'ekf_num_points_mean_time.png');
% Time for one point
figure();
scatter(num_points_vec, ekf_mean_time_vec./num_points_vec/(10^(-3)))
xlabel('Number of Points');
ylabel('Time Elapsed [ms]');
title('EKF:Number of Points versus Time Elapsed for Each Point');

saveas(gcf,'ekf_num_points_single_op_time.png');
average_time_per_point = mean(ekf_mean_time_vec./num_points_vec/(10^(-3)));
%% UKF
clc;
close all;
num_points_vec = [100, 1000, 10000, 15000, 20000, 25000];
ukf_time_elapsed =   [  0.052070, 0.461360, 4.408105, 6.678124, 8.816481, 11.036092;
                    0.050724, 0.455492, 4.447681, 6.666712, 8.731272, 11.117507;
                    0.052175, 0.454480, 4.532233, 6.699965, 8.676504, 11.146030 ];
ukf_mean_time_vec = mean(ukf_time_elapsed)
figure()
scatter(num_points_vec, ukf_mean_time_vec)
title('UKF: Number of Points versus Mean Time');
xlabel('Number of Points');
ylabel('Time Elapsed (s)');
saveas(gcf,'ukf_num_points_mean_time.png');
% Time for one point
figure();
scatter(num_points_vec, ukf_mean_time_vec./num_points_vec/(10^(-3)))
title('UKF:Number of Points versus Time Elapsed for Each Point');

xlabel('Number of Points');
ylabel('Time Elapsed [ms]');
saveas(gcf,'ukf_num_points_single_op_time.png');
average_time_per_point = mean(ukf_mean_time_vec./num_points_vec/(10^(-3)));
%% EKF, UKF Comparison
figure()
hold on
plot(num_points_vec,ekf_mean_time_vec)
plot(num_points_vec,ukf_mean_time_vec)
title('EKF v UKF Mean Time Versus Number of Points');
xlabel('Number of Points')
ylabel('Time [s]');
legend('EKF','UKF')
hold off
saveas(gcf,'ekf_ukf_comparison');