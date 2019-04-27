%% testing functions
clear all; clc; close all;
%% testing meas_vec.m
working_directory = % FILL THIS IN
accel_location = % FILL THIS IN
gps_location = % FILL THIS IN
gyro_location = % FILL THIS IN
[Y] = meas_vec(working_directory, gps_location, accel_location, gyro_location);