function [cur_data_vec, new_previndex_vec] = ethsense(cur_time, previndex_vec)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% onboardgps = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
rawaccel = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\rawaccel.mat');
found1 = find(rawaccel.RawAccel(previndex_vec(2):size(rawaccel.RawAccel,1),1) == int64(cur_time));
new_previndex_vec = previndex_vec;
[m1,n1] = size(found1);
if(m1 == 1)
    disp(cur_time)
    disp('found accel');
    dvaccel = [rawaccel.RawAccel(found1(1),2:4)];
    new_previndex_vec(2) = found1(1);
else
    disp(cur_time)
    disp('going with last accel');
    dvaccel = [rawaccel.RawAccel(previndex_vec(2),2:4)];
end
cur_data_vec = dvaccel;
return
% cur_data_vec, previndex_vec