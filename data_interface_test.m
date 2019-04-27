clc; clear all;
load('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\rawaccel.mat')
load('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
tic
cur_time = RawAccel(1:500,1);
rawaccel = RawAccel;
rawgps = OnboardGPS;
rawgyro = RawAccel;
toc
%%
clc;
previndex_vec = [1,1,1]
tic
for i = cur_time'
%     disp(i)
    [cur_data_vec, new_previndex_vec] = ethsense(i, previndex_vec, rawaccel, rawgps, rawgyro);
    disp('cur_data_vec'); disp(cur_data_vec);
%     disp('new_previndex_vec'); disp(new_previndex_vec);
%     disp(isequal(previndex_vec,new_previndex_vec));
    if(~isequal(previndex_vec,new_previndex_vec))
        disp('current data:');
        disp(cur_data_vec);
        disp(new_previndex_vec);
        previndex_vec = new_previndex_vec;
    else
        previndex_vec = new_previndex_vec;
    end 
end
toc