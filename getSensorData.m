function [objectActualLocation_x,objectActualLocation_y,Y,K] = SensorData()
%SensorData Creates dummy sensor data
i=0:0.01:1;
leg1_y = i*1.9+(1-i)*4.9;
leg1_x = 1.2*ones(1,101);

leg2_y = 1.9*ones(1,101);
leg2_x = i*4.0+(1-i)*1.2;

leg3_y = i*5.0+(1-i)*1.9;
leg3_x = 4.0*ones(1,101);

leg4_y = 5.0*ones(1,101);
leg4_x = i*3.0+(1-i)*4.0;

objectActualLocation_x = [leg1_x leg2_x leg3_x leg4_x]';
objectActualLocation_y = [leg1_y leg2_y leg3_y leg4_y]';


%objectActualLocation_x = [1.2,1.2,1.2,1.2,2.0,3.0,4.0,4.0,4.0,4.0,3.0]'; 
%objectActualLocation_y = [4.9,3.9,2.9,1.9,1.9,1.9,1.9,3.0,4.0,5.0,5.0]';

K = size(objectActualLocation_x,1);

Y = ones(9,K);





%{
%Sensor 1 Location
x1 = 1;
y1 = 1;

%Sensor 2 Location
x2 = 5;
y2 = 7;

theta1 = atan((objectActualLocation_x - x1)./(objectActualLocation_y - y1));
theta2 = atan((x2 - objectActualLocation_x)./(y2 - objectActualLocation_y));

%Add noise
theta1 = awgn(theta1,30);
theta2 = awgn(theta2,30);
%}
end

