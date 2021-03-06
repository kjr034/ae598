function [cur_data_vec, new_previndex_vec_] = ethsense(cur_time,...
    previndex_vec, rawaccel, rawgps, rawgyro)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% onboardgps = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
%     disp(previndex_vec(2))
%     rawaccel = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\rawaccel.mat');
%   FIND THE STUFF HERE!
    [m_accel,n_accel,v] = find(rawaccel(:,1) == int64(cur_time));
    [m_gps,n_gps] = find(rawgps(:,1) < int64(cur_time) & rawgps(:,1) >= rawaccel(previndex_vec(2),1));
    [m_gyro,n_gyro] = find(rawgyro(:,1) < int64(cur_time) & rawgyro(:,1) >= rawaccel(previndex_vec(2),1));
%   IF FOUND, do stuff...     
    new_previndex_vec_ = previndex_vec;
    
%     [m_accel,n_accel] = size(found_accel);
%     [m_gps,n_gps] = size(found_gps)
%     [m_gyro,n_gyro] = size(found_gyro);
%     disp('m_accel, m_gps, m_gyro');
%     disp([m_accel, m_gps, m_gyro]);
    if(size(m_accel,1) > 0)
        dvaccel = rawaccel(m_accel(1),2:3);
        disp(rawaccel(m_accel(1),1))
        new_previndex_vec_(2) = m_accel(1);
    else
        disp('i shouldn"t be here');
        dvaccel = [rawaccel(new_previndex_vec_(2),2:3)];
    end
    if(size(m_gps,1) > 0)
        disp(rawaccel(m_accel(1),1))
        dvgps = [rawgps(m_gps(1),4:-1:3),rawgps(m_gps(1),12:-1:11)];
%         dvgps = [rawgps(m_gps(1),3:4)',rawgps(m_gps(1),6:8)'];
        new_previndex_vec_(1) = m_gps(1);
    else
        dvgps = [rawgps(new_previndex_vec_(1),4:-1:3),...
            rawgps(new_previndex_vec_(1),12:-1:11)];
%         dvgps = [rawgps(prev_index(1),3:4)',rawgps(prev_index(1),6:8)'];
    end
    if(size(m_gyro,1) > 0)
        dvgyro = [rawgyro(m_gyro(1),5)];
        new_previndex_vec_(3) = m_gyro(1);
    else
        dvgyro = [rawgyro(new_previndex_vec_(3),5)];
    end
    cur_data_vec = [dvgps, dvaccel, dvgyro];
%     disp(new_previndex_vec_);
return