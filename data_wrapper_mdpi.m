%% Load!
close all; clc;
load('\\client\c$\Users\KJR03\OneDrive\Documents\UIUC\SP_19\AE598\Final_Project\MDPI Data\V1.5_wp_10_Flight01.mat')
%% GET IMU data
clc;
line = 761:1:1000
%%
clc;
%%
NUM_VALS = 8
IMU_DIM = 7
synch_dat = 1:1:size(line,1)
disp(synch_dat)
full_mat = zeros(size(synch_dat,2),NUM_VALS)
p = 1
for i = line
    disp(i)
    found1 = find(IMU(:,1) == int64(i));
    [m,n] = size(found1);
    if m == 1
        full_mat(i,:) = [p, IMU(found1(1),2), IMU(found1(1),4:9)]
    else
        full_mat(i,:) = [p, IMU(i-1,2), IMU(i-1,4:9)]
    end
    p = p+1
end
