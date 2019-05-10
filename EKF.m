% Note: Y is gener ated via the function SensorData(). Because we can
% essentially go through this section of the data with this one function,
% we'll be preprocessing the data within SensorData(). Note also that the
% overall actual locations (objectActualLocation_x,_y) willl be found via
% the "GroundTruth" dataset given by ETHZ. The dataprocessing for the
% current dataset shoulud be about 1 minute. If the data takes longer than
% this to load, kill the program.
clear all;
N = 12048
% [objectActualLocation_x,objectActualLocation_y,Y,K] = getSensorData(N);
[objectActualLocation_x,objectActualLocation_y,Y,K] = getSensorData2(N);
%%
f = 32
[x_input,y_input,f]=ll2utm([Y(:,1),Y(:,2)]) % for MPsense
% [x_input,y_input,f]=ll2utm([Y(:,2),Y(:,1)]) % for ethsense
% lla_output = lla2flat([Y(:,1:2) zeros(size(Y(:,1)))], [0 32], -5, 100)

%%
t_start = 1
%Initial conditions
x_0 = 0;
y_0 = 0;
V_x_0 = 0;
V_y_0 = 0;
a_x_0 = 0;
a_y_0 = 0;  
psi_0 = 0;
omega_0 =0;

x = zeros(8,K);
x_east(:,1) = [x_0,V_x_0,a_x_0]';
x_north(:,1) = [y_0,V_y_0,a_y_0]';
% x(:,1) = [x_east(:,1); x_north(:,1);psi_0;omega_0];
x_ekf(:,1) = [x_0, y_0, V_x_0, V_y_0, a_x_0, a_y_0, psi_0, omega_0]';
% Ts = 0.01; %10 Hz


%EKF
% Essentially: 
% obj = extendedKalmanFilter(x_k, y_k, x_init);
obj = extendedKalmanFilter(@stateTransitionFunc,@measurementFcn,x(:,1));
obj.MeasurementNoise = diag([1; 1; 1; 1; 1; 1; 01]);
% for the ETH dataset, the t_start = RawAccel(1,1) --> this is in
% microseconds
tic
for k =t_start:12048
    y = [x_input(k,1)-x_input(1,1) y_input(k,1)-y_input(1,1) Y(k,3:7)]'; % this needs to be changed to [x_input(k,1) y_input(k,1) ...]
    [CorrectedState,CorrectedStateCovariance] = correct(obj,y);
    [PredictedState,PredictedStateCovariance] = predict(obj);
    x_ekf(:,k) = obj.State(:,1);
end
toc


function x_k = stateTransitionFunc(x_k_minus_1)
    Ts = 0.1; %10 Hz
    A = [1 0 Ts 0  0.5*Ts^2       0 0 0; ...
         0 1 0  Ts    0       0.5*Ts^2 0 0; ...
         0 0 1  0     Ts          0 0 0; ...
         0 0 0  1     0           Ts 0 0; ...
         0 0 0  0     1           0 0 0; ...
         0 0 0  0     0           1 0 0; ...
         0 0 0  0     0           0 1 Ts; ...
         0 0 0  0     0           0 0 1];
    x_k = A*x_k_minus_1;
end

function y_k = measurementFcn(x_k)
    
    %GPS
%     lla = lla2flat([ x_k(1) x_k(2) 0 ], [0 32], 5, -100);
    y_k(1) = x_k(1); %LongGpsE
    y_k(2) = x_k(2); %LatGpsN
    y_k(3) = x_k(3); %VelNedE
    y_k(4) = x_k(4); %VelNedN
    
    %IMU
    psi = x_k(7); 
    DCM = [cos(psi) -sin(psi);
           sin(psi) cos(psi)];
    a_body = DCM*[x_k(5);x_k(6)];
    y_k(6) = a_body(1); %a_x_body
    y_k(7) = a_body(2); %a_y_body
    y_k(5) = x_k(8); %omega_z_body
    
%     lla = flat2lla( [ x_k(1) x_k(2) 0 ], [0 45], 5, -100);
% 
%     [xyz, h, dec, dip, f] = wrldmagm(0, lla(1), lla(2) , decyear(2019,4,7));
%     y_k(8) = xyz(1); %magx
%     y_k(9) = xyz(2); %magy
end



%%
%{

function y_k = measuremPS Measurements
    x_k(1) = LongGpsE; %x
    x_k(2) = LatGpsN; %y
    x_k(3) = VelNedE; %Vx
    x_k(4) = VelNedN; %VyentFcn(x_k)
    syms y_sym;
    y_k = solve(h_inverse(y_sym)-x_k,y_sym);
    y_k = double(y_k);
end

function x_k = h_inverse(y_k)
    LongGpsE = y_k(1);
    LatGpsN = y_k(2);
    VelNedE = y_k(3);
    VelNedN = y_k(4);
    magx = y_k(5);
    magy = y_k(6);
    omega_z_body = y_k(7);
    a_x_body = y_k(8);
    a_y_body = y_k(9);
    
    % GPS Measurements
    x_k(1) = LongGpsE; %x
    x_k(2) = LatGpsN; %y
    x_k(3) = VelNedE; %Vx
    x_k(4) = VelNedN; %Vy
    
    % IMU Measurements
    [yaw,Rh] = Algorithm1_2(magx,magy,omega_z_body,0);
    
     a = Rh*[a_x_body;a_y_body]; %ax;ay
     x_k(5) = a(1); %ax
     x_k(6) = a(2); %ay
    
end

function [yaw,Rh] = Algorithm1_2(magx,magy,gyro_z,Rh)
    mnorm = norm([magx magy]); % mx(k) my(k) from magnetometer
    mx = magx/mnorm; my =mag    y_k(4) = x_k(4); %VelNedN
y/mnorm;
    psi = atan2(-my,mx);
    DCM = [cos(psi) -sin(psi);
           sin(psi) cos(psi)];
    YAW = 180/pi*psi; %YAW from magnetometer
    %{
    gyro = [ 0 -gyro_z; % wz(k) from z axis gyro
            gyro_z 0 ];
    omaga = 0.5*(Rh'*DCM - DCM'*Rh);
    OM = gyro + kp*omaga; % kp >= 1, gain constant
    nOM = norm(OM); %For exact solution
    A = eye(2) - OM.*sin(nOM*Ts)./nOM + (OM^2).*(1-cos(nOM*Ts))/nOM^2;
    Rh = Rh*A';
    yaw = 180/pi*atan2(Rh(2,    y_k(4) = x_k(4); %VelNedN
1),Rh(1,1)); %yaw, Complementary Filter
    %}
    yaw = YAW;
    Rh = DCM;
end
%}


%{
function y_k = measurementFcn(x_k)
    %Sensor 1 Location
    x1 = 1;
    y1 = 1;

    %Sensor 2 Location
    x2 = 5;
    y2 = 7;

    theta1 = atan((x_k(1) - x1)./(x_k(4) - y1));
    theta2 = atan((x2 - x_k(1))./(y2 - x_k(4)));
    
    y_k = [theta1;theta2];
end
%}