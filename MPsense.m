function [cur_data_vec, new_previndex_vec_] = MPsense(cur_time,...
    previndex_vec, xacc, yacc, zgyro, vxgps, vygps,lat, lon)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% onboardgps = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\onboardgps.mat')
%     disp(previndex_vec(2))
%     rawaccel = matfile('\\client\c$\Users\KJR03\AE598\ae598\AGZ_subset\Log_Files\rawaccel.mat');
%   FIND THE STUFF HERE!
    disp('MPsense')
    disp('cur_time')
    disp(cur_time-7.375556909351505e+05)
    N = size(xacc,1)
    size(xacc(:,1))

    [m_xaccel,n_xaccel,v] = find(xacc(1:N,1) == (cur_time));
%     [m_yaccel,n_yaccel,v] = find(yacc_mavlink_raw_imu_t(:,1) == int64(cur_time));
    [m_lat,n_lat] = find(lat(1:N,1) < (cur_time) & lat(1:N,1) >= xacc(previndex_vec(2),1));
%     [m_lon,n_lon] = find(lon_mavlink_global_position_int_t(:,1) < int64(cur_time) & lon_mavlink_global_position_int_t(:,1) >= xacc_mavlink_raw_imu_t(previndex_vec(2),1));
%     [m_gyro,n_gyro] = find(rawgyro(:,1) < int64(cur_time) & rawgyro(:,1) >= rawaccel(previndex_vec(2),1));
%   IF FOUND, do stuff...     
    new_previndex_vec_ = previndex_vec;
    
%     [m_accel,n_accel] = size(found_accel);
%     [m_gps,n_gps] = size(found_gps)
%     [m_gyro,n_gyro] = size(found_gyro);
%     disp('m_accel, m_gps, m_gyro');
%     disp([m_accel, m_gps, m_gyro]);
    if(size(m_xaccel,1) > 0)
        dvaccel = [xacc(m_xaccel(1),2)*9.8/1000, yacc(m_xaccel(1),2)*9.8/1000]
        dvgyro = zgyro(m_xaccel(1),2)
        new_previndex_vec_(2) = m_xaccel(1);
    else
        disp('i shouldn"t be here');
        dvaccel = [xacc(new_previndex_vec_(2),2), yacc(new_previndex_vec_(2),2)];
        dvgyro = zgyro(new_previndex_vec_(2),2)
    end
    if(size(m_lat,1) > 0)
        dvgps = [lat(m_lat(1),2)/(10^(6)),lon(m_lat(1),2)/(10^(6)), vxgps(m_lat(1),2), vygps(m_lat(1),2)];
%         dvgps = [rawgps(m_gps(1),3:4)',rawgps(m_gps(1),6:8)'];
        new_previndex_vec_(1) = m_lat(1);
    else
        dvgps = [lat(new_previndex_vec_(1),2)/(10^(7)), ...
                 lon(new_previndex_vec_(1),2)/(10^(7)), ...
                 vxgps(new_previndex_vec_(1),2), ...
                 vygps(new_previndex_vec_(1),2)]
%         dvgps = [rawgps(prev_index(1),3:4)',rawgps(prev_index(1),6:8)'];
    end
    cur_data_vec = [dvgps, dvaccel, dvgyro];
%     disp(cur_data_vec)
%     disp(dvgps)
%     disp(dvaccel)
%     disp(dvgyro)
%     disp(new_previndex_vec_);
return
