%% testing functions
clear all; clc; close all;
%% testing meas_vec.m
working_directory = '\\client\c$\Users\KJR03\AE598\ae598\' % FILL THIS IN
accel_location = 'AGZ_subset\Log_Files\rawaccel.mat' % FILL THIS IN
gps_location = 'AGZ_subset\Log_Files\rawgyro.mat'% FILL THIS IN
gyro_location = 'AGZ_subset\Log_Files\onboardgps.mat'% FILL THIS IN
[Y] = meas_vec(working_directory, gps_location, accel_location, gyro_location);
