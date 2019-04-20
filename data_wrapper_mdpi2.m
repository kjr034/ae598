%% Load!
close all; clc;
load('\\client\c$\Users\KJR03\OneDrive\Documents\UIUC\SP_19\AE598\Final_Project\MDPI Data\V1.5_wp_10_Flight01.mat')
%% GET IMU data
clc;
line = 761:1:3400
%%
clc;
%%
tic
NUM_VALS = 9;
IMU_DIM = 7;
synch_dat = 1:1:size(line,1);
% disp(synch_dat);
full_mat = zeros(size(synch_dat,2),NUM_VALS);
p = 1;
a = 1;
b = 1;
for i = line
%     disp(i)
    found1 = find(IMU(:,1) == int64(i));
    found2 = find(GPS(:,1) == int64(i));
    [m,n] = size(found1);
    [m2,n2] = size(found2);
    if m == 1 || m2 ==1
        if(m ==1  && m2 ==1)
                full_mat(p,:) = [p, IMU(found1(1),2), IMU(found1(1),3:4), GPS(found2(1),8:9),GPS(found2(1),11)];
                a = found1(1);
                b = found2(1);
                
        elseif(m==1 && m2 == 0)
                full_mat(p,:) = [p, IMU(found1(1),2), IMU(found1(1),3:4),IMU(found1(1),6:7),GPS(b,8:9),GPS(b,11)];
                a = found1(1);
        else
            full_mat(p,:) = [p, IMU(a,2), IMU(a,3:4),IMU(a,6:7),GPS(found2(1),8:9),GPS(found2(1),11)];
            b = found2(1);  
        end
    else  
        full_mat(p,:) = [p, IMU(a,2), IMU(a,3:4),IMU(a,6:7),GPS(b,8:9),GPS(b,11)];
    end
    
    p = p+1;
end
toc