%% ETH ZURICH DATA SHIPPING/RECEIVING
clear all; clc; close all;
% load GPS data
% load('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
% onboardgps = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
rawaccel = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\rawaccel.mat')
% load IMU data
% load('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\rawaccel.mat')
% load Ground Truth data (?)

%% Clear again
clc; close all;
%% Creating a line of numbers starting with 7 E-6:
% These values will hold the last time step value for ACCEL, GPS, and GYRO
% data. By storing the last value, there will be some simple combinational
% logic to determine whether or not the next value in a line of values can
% be used. Essentially, by doing this, we ensure that there is quick access
% to the next variable. 
% Format: [GPS_time, Accel_time, gyro_time]
last_val_timestep = zeros(3,1) % initialize temporary storage
cur_val_timestep = zeros(3,1) % initialize temporary storage
next_val_timestep = zeros(3,1) % initialize temporary storage
%% Populate the appropriate value at the appropriate time. 
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% from GPS: want   t, Lat, Lon, v_n, v_e --> indexed from 1
% onboardGPS num:  1,   3,   4,  11,  12,
% IMU- Accel want: t, x, y, z
% RawAccel num:    1, 3, 4, 5
% IMU- Gyro want:  t, x, y, z --> really only care about z in basic EKF
% RawGyro num:     1, 3, 4, 5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
IMU_VALS = 6; % three for RawAccel, three for RawGyro
NUM_VALS = GPS_VALS + IMU_VALS % for this particular problem, should be 10
% for testing, use just GPS values
data_vec = zeros(size(time_vec,2),NUM_Vals);
% data_vec = zeros(size(time_vec,1),NUM_VALS);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get the DATA
p = 1;

GPS_last = 1;
ACCEL_last = 1;
% GYRO_last = 1;

m1 = 1;
n1 = 1;
dvGPS = zeros(1,GPS_VALS) % 1x5 GPS vector size (allocate mem)
% dvACCEL = zeros(IMU_VALS/2) % ACCEL vector size (allocate mem)
% dvGYRO = zeros(IMU_VALS/2) % GYRO vector size (allocate mem)
%%
clear all;
%%
prev = [2,1,1]
clc;
tic
i = 9290922
%     disp(i); % for debugging
%     disp(p);
% get the position of the GPS, Accel, Gyro values
found1 = find(rawaccel.RawAccel(:,1) == int64(i));
%     found2 = find(RawAccel(:,1) == int65(i));
%     found3 = find(RawGyro(:,1) == int65(i));
%     The location of the objects:
[m1,n1] = size(found1); 
toc
%     [m2,n2] = size(found2);
%     [m3, n3] = size(found3);

% if at least one found:
% Checking for whether GPS values are actually in the array,
% If not, then use the last values recorded
if(m1 == 1)
    disp(i)
    disp('found');
    dvaccel = [rawaccel.RawAccel(found1(1),2:4)];
else
    disp(i)
    disp('not found');
    if(p > 1)
        dvaccel = rawaccel.RawAccel(prev(2)); % get the GPS vals from last time
    else 
        dvGPS = accel(GPS_VALS,1);  % else populate with zeros
    end
end
%%
clc;
tic
cur_time = rawaccel.RawAccel(:,1);
toc
%%
clc; clear all;
load('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\rawaccel.mat')
tic
cur_time = RawAccel(:,1);
rawaccel = RawAccel;
rawgps = RawAccel;
rawgyro = RawAccel;
toc
%%
clc;
previndex_vec = [1,1,1]
tic
for i = cur_time'
%     disp(i)
    [cur_data_vec, new_previndex_vec] = ethsense(i, previndex_vec, rawaccel, rawgps, rawgyro);
%     disp('cur_data_vec'); disp(cur_data_vec);
%     disp('new_previndex_vec'); disp(new_previndex_vec);
%     disp(isequal(previndex_vec,new_previndex_vec));
    if(~isequal(previndex_vec,new_previndex_vec))
%         disp('current data:');
%         disp(cur_data_vec);
%         disp(new_previndex_vec);
        previndex_vec = new_previndex_vec;
    else
        previndex_vec = new_previndex_vec;
    end 
end
toc