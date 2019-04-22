%% ETH ZURICH DATA SHIPPING/RECEIVING
clear all; clc; close all;
% load GPS data
load('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
% load IMU data
load('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
% load Ground Truth data (?)

%% Clear again
clc; close all;
%% Creating a line of numbers starting with 7 E-6:
time_vec_start = OnboardGPS(1,1) % this gets the number the GPS starts with
time_vec_end = OnboardGPS(1,1)*1.1 % for testing, *1.2, otherwise *1.5 --> larger test, end of mat for full test
time_vec = time_vec_start:33333:time_vec_end; % create the vector

%% Populate the appropriate value at the appropriate time. 
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% from GPS: want   t, Lat, Lon, v_n, v_e --> indexed from 1
% onboardGPS num:  1,   3,   4,  11,  12,
GPS_VALS = 4; % number of values to be put into the obv vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IMU- Accel want: t, x, y, z
% RawAccel num:    1, 3, 4, 5
% IMU- Gyro want:  t, x, y, z --> really only care about z in basic EKF
% RawGyro num:     1, 3, 4, 5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
IMU_VALS = 6; % three for RawAccel, three for RawGyro
NUM_VALS = GPS_VALS + IMU_VALS % for this particular problem, should be 10
% for testing, use just GPS values
data_vec = zeros(size(time_vec,2),GPS_VALS+1);
% data_vec = zeros(size(time_vec,1),NUM_VALS);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get the DATA
p = 1;

GPS_last = 1;
% ACCEL_last = 1;
% GYRO_last = 1;

m1 = 1;
n1 = 1;
dvGPS = zeros(1,GPS_VALS) % 1x5 GPS vector size (allocate mem)
% dvACCEL = zeros(IMU_VALS/2) % ACCEL vector size (allocate mem)
% dvGYRO = zeros(IMU_VALS/2) % GYRO vector size (allocate mem)
tic
for i = time_vec
%     disp(i); % for debugging
%     disp(p);
% get the position of the GPS, Accel, Gyro values
    found1 = find(OnboardGPS(:,1) == int64(i));
%     found2 = find(RawAccel(:,1) == int65(i));
%     found3 = find(RawGyro(:,1) == int65(i));
%     The location of the objects:
    [m1,n1] = size(found1); 
%     [m2,n2] = size(found2);
%     [m3, n3] = size(found3);
    
    % if at least one found:
% Checking for whether GPS values are actually in the array,
% If not, then use the last values recorded
    if(m1 == 1)
        disp(i)
        disp('found');
        dvGPS = [OnboardGPS(found1(1),3:4), OnboardGPS(found1(1),11:12)];
        GPS_last   = found1(1);
    else
        disp(i)
        disp('not found');
        if(p > 1)
            dvGPS = data_vec(p-1,2:5); % get the GPS vals from last time
        else 
            dvGPS = zeros(GPS_VALS,1);  % else populate with zeros
        end
    end
% % Checking for whether ACCEL values are actually in the array,
% % If not, then use the last values recorded
%     if(m2 ==1)
%             dvACCEL = [OnboardGPS(found1(1),3:4), OnboardGPS(found1(1),11:12)];
%             ACCEL_last   = found2(1);
%     else
%         if(p > 1)
%             dvACCEL = data_vec(p-1,6:8)
%         else 
%             dvACCEL = zeros(IMU_VALS/2,1)  % else populate with zeros
%         end
%     end
% % Checking for whether GYRO values are actually in the array,
% % If not, then use the last values recorded
%     if(m3 ==1)
%             dvGYRO = [OnboardGPS(found1(1),3:4), OnboardGPS(found1(1),11:12)];
%             GYRO_last   = found1(1);
%     else
%         if(p > 1)
%             dvGYRO = data_vec(p-1,9:11)
%         else 
%             dvGYRO = zeros(IMU_VALS/2,1) % else populate with zeros
%         end 
%     end
%     data_vec_(p,:) = [p, dvGPS, dvACCEL, dvGYRO]; 
    data_vec(p,:) = [p, dvGPS];
    p = p+1;
end
toc