function [final_data_vec] = meas_vec2(N,working_directory, data_location)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% On Linux subsystem:
% adding the values we want:
your_working_directory = working_directory
disp('here')
disp(strcat(your_working_directory,data_location))
load(strcat(your_working_directory,data_location));

% load(strcat(your_working_directory,pos_gt))
% setting current time
cur_time = xacc_mavlink_raw_imu_t(1:N,1);
% setting variable namesyour_working_directory = '/home/johnkan2/ae598/final_proj'
zgyro = zgyro_mavlink_raw_imu_t;
xacc = xacc_mavlink_raw_imu_t;
yacc = yacc_mavlink_raw_imu_t;
vxgps = vx_mavlink_global_position_int_t
vygps = vy_mavlink_global_position_int_t
lat = lat_mavlink_global_position_int_t
lon = lon_mavlink_global_position_int_t
disp(xacc)
%%
previndex_vec = [1,1,1]
tic
final_data_vec = zeros(N,7);
n = 1
for i = cur_time'
    disp('n')
    disp(n)
    
    
    [final_data_vec(n,:), new_previndex_vec] = MPsense(i, previndex_vec, xacc, yacc, zgyro, vxgps, vygps,lat, lon);

    if(~isequal(previndex_vec,new_previndex_vec))
        previndex_vec = new_previndex_vec;
    else
        previndex_vec = new_previndex_vec;
    end 
    n=n+1;
end
toc
end