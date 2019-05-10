%% EKF
close all;
num_points_vec = [100, 1000, 10000, 15000, 20000, 25000];
time_elapsed =   [  0.055543, 0.546664, 4.619367, 6.915015, 9.091876, 11.214172;
                    0.050048, 0.464550, 4.456990, 6.849361, 8.968956, 11.337758;
                    0.050605, 0.503496, 4.548662, 6.799907, 8.923555, 11.326292 ];
mean_time_vec = mean(time_elapsed)
figure()
scatter(num_points_vec, mean_time_vec)
title('EKF: Number of Points versus Mean Time');
xlabel('Number of Points');
ylabel('Time Elapsed (s)');
saveas(gcf,'ekf_num_points_mean_time.png');
% Time for one point
figure();
title('EKF:Number of Points versus Time Elapsed for Each Point');
scatter(num_points_vec, mean_time_vec./num_points_vec/(10^(-3)))
xlabel('Number of Points');
ylabel('Time Elapsed [ms]');
saveas(gcf,'ekf_num_points_single_op_time.png');
average_time_per_point = mean(mean_time_vec./num_points_vec/(10^(-3)));
% UKF
clc; clear all;
close all;
num_points_vec = [100, 1000, 10000, 15000, 20000, 25000];
time_elapsed =   [  0.055543, 0.546664, 4.619367, 6.915015, 9.091876, 11.214172;
                    0.050048, 0.464550, 4.456990, 6.849361, 8.968956, 11.337758;
                    0.050605, 0.503496, 4.548662, 6.799907, 8.923555, 11.326292 ];
mean_time_vec = mean(time_elapsed)
figure()
scatter(num_points_vec, mean_time_vec)
title('EKF: Number of Points versus Mean Time');
xlabel('Number of Points');
ylabel('Time Elapsed (s)');
saveas(gcf,'ekf_num_points_mean_time.png');
% Time for one point
figure();
title('EKF:Number of Points versus Time Elapsed for Each Point');
scatter(num_points_vec, mean_time_vec./num_points_vec/(10^(-3)))
xlabel('Number of Points');
ylabel('Time Elapsed [ms]');
saveas(gcf,'ekf_num_points_single_op_time.png');
average_time_per_point = mean(mean_time_vec./num_points_vec/(10^(-3)));