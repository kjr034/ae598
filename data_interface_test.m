clc; clear all;
N = 27
% On Linux subsystem:

your_working_directory = '\\client\c$\Users\KJR03\AE598\ae598\' % FILL THIS IN
accel_location = 'AGZ_subset\Log_Files\rawaccel.mat' % FILL THIS IN
gps_location = 'AGZ_subset\Log_Files\rawgyro.mat'% FILL THIS IN
gyro_location = 'AGZ_subset\Log_Files\onboardgps.mat'% FILL THIS IN

load(strcat(your_working_directory,accel_location));
load(strcat(your_working_directory,gyro_location));
load(strcat(your_working_directory,gps_location));
tic
cur_time = RawAccel(1:N,1);
rawaccel = RawAccel;
rawgps = OnboardGPS;
rawgyro = RawGyro;
toc
%%
clc;
previndex_vec = [1,1,1]

final_data_vec = zeros(N,7);
n = 1
tic
for i = cur_time'
    disp(i)
    [final_data_vec(n,:), new_previndex_vec] = ethsense(i, previndex_vec, rawaccel, rawgps, rawgyro);
    if(~isequal(previndex_vec,new_previndex_vec))
        previndex_vec = new_previndex_vec;
    else
        previndex_vec = new_previndex_vec;
    end 
    n=n+1;
end
toc
%%
N = 100
[objectActualLocation_x,objectActualLocation_y,Y,K] = getSensorData(N)