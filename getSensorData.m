function [objectActualLocation_x,objectActualLocation_y,Y,K] = getSensorData()
% This function will return the groudntruth locations (x and y), as well as
% the measurement functions

working_directory = % FILL THIS IN
accel_location = % FILL THIS IN
gps_location = % FILL THIS IN
gyro_location = % FILL THIS IN
[Y] = meas_vec(working_directory, gps_location, accel_location, gyro_location)


end

