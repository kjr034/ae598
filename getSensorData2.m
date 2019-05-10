function [objectActualLocation_x,objectActualLocation_y,Y,K] = getSensorData2(N)
% This function will return the groudntruth locations (x and y), as well as
% the measurement functions
working_directory = '/home/johnkan2/ae598/final_proj/ae598/'; % FILL THIS IN
data_location = 'MP_subset/MP_real_dataset.mat'; % FILL THIS IN


% working_directory = '\\client\c$\Users\KJR03\AE598\ae598\'; % FILL THIS IN
% accel_location = 'AGZ_subset\Log_Files\rawaccel.mat'; % FILL THIS IN
% gps_location = 'AGZ_subset\Log_Files\rawgyro.mat';% FILL THIS IN
% gyro_location = 'AGZ_subset\Log_Files\onboardgps.mat';% FILL THIS IN
% disp('here')
[Y] = meas_vec2(N,working_directory, data_location);
K = size(Y,1);
objectActualLocation_x = zeros(size(Y,1),1);
objectActualLocation_y = zeros(size(Y,1),1);
end