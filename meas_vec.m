function [final_data_vec] = meas_vec(working_directory, gps_location, accel_location, gyro_location)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
clc;

% On Linux subsystem:
% adding the values we want:
your_working_directory = working_directory
load(strcat(your_working_directory,accel_location));
load(strcat(your_working_directory,gyro_location));
load(strcat(your_working_directory,gps_location));
% setting current time
cur_time = RawAccel(:,1);
% setting variable namesyour_working_directory = '/home/johnkan2/ae598/final_proj'
rawaccel = RawAccel;
rawgps = OnboardGPS;
rawgyro = RawGyro;
%%
clc;
previndex_vec = [1,1,1]
tic
final_data_vec = zeros(27048,8);
n = 1
for i = cur_time'
    disp(i)
    [final_data_vec(n,:), new_previndex_vec] = ethsense(i, previndex_vec, rawaccel, rawgps, rawgyro);
%     disp('cur_data_vec'); disp(cur_data_vec);
    
%     disp('new_previndex_vec'); disp(new_previndex_vec);
%     disp(isequal(previndex_vec,new_previndex_vec));\
    disp(rawgps(previndex_vec(1),1))
    disp(rawaccel(previndex_vec(2),1))
    disp(rawgyro(previndex_vec(3),1))
    if(~isequal(previndex_vec,new_previndex_vec))
        disp('current data:');
        disp(final_data_vec(n,:));
        disp(new_previndex_vec);
        previndex_vec = new_previndex_vec;
    else
        previndex_vec = new_previndex_vec;
    end 
    n=n+1;
end
toc
end

