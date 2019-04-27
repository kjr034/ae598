clc; clear all;

% On Linux subsystem:
your_working_directory = 

load(strcat(your_working_directory,'ae598/AGZ_subset/Log_Files/rawaccel.mat'));
load(strcat(your_working_directory,'ae598/AGZ_subset/Log_Files/onboardgps.mat'));
load(strcat(your_working_directory,'ae598/AGZ_subset/Log_Files/RawGyro.mat'));
tic
cur_time = RawAccel(1:50,1);
rawaccel = RawAccel;
rawgps = OnboardGPS;
rawgyro = RawGyro;
toc
%%
clc;
previndex_vec = [1,1,1]
tic
for i = cur_time'
    disp(i)
    [cur_data_vec, new_previndex_vec] = ethsense(i, previndex_vec, rawaccel, rawgps, rawgyro);
    disp('cur_data_vec'); disp(cur_data_vec);
    
%     disp('new_previndex_vec'); disp(new_previndex_vec);
%     disp(isequal(previndex_vec,new_previndex_vec));\
    disp(rawgps(previndex_vec(1),1))
    disp(rawaccel(previndex_vec(2),1))
    disp(rawgyro(previndex_vec(3),1))
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