function [objectActualLocation_x,objectActualLocation_y,Y,K] = getSensorData()
% This function will return the groudntruth locations (x and y), as well as
% the measurement functions
working_directory = '\\client\c$\Users\KJR03\AE598\ae598\'; % FILL THIS IN
accel_location = 'AGZ_subset\Log_Files\rawaccel.mat'; % FILL THIS IN
gps_location = 'AGZ_subset\Log_Files\rawgyro.mat';% FILL THIS IN
gyro_location = 'AGZ_subset\Log_Files\onboardgps.mat';% FILL THIS IN
[Y] = meas_vec(working_directory, gps_location,accel_location, gyro_location);
K = size(Y,1);
objectActualLocation_x = zeros(size(Y,1),1);
objectActualLocation_y = zeros(size(Y,1),1);
end


