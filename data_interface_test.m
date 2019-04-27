clc; clear all;

% On Linux subsystem:

your_working_directory = % FILL THIS IN
accel_location = % FILL THIS IN
gps_location = % FILL THIS IN
gyro_location = % FILL THIS IN

load(strcat(your_working_directory,accel_location));
load(strcat(your_working_directory,gyro_location));
load(strcat(your_working_directory,gps_location));
tic
cur_time = RawAccel(:,1);
rawaccel = RawAccel;
rawgps = OnboardGPS;
rawgyro = RawGyro;
toc
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