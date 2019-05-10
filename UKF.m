% clear all;
N = 27000
[objectActualLocation_x,objectActualLocation_y,Y,K] = getSensorData(N);
%% Start of UKF
clc;
x_ukf = zeros(8,K);
x_ukf(:,1) = x(:,1);
obj_ukf = unscentedKalmanFilter(@stateTransitionFunc,@measurementFcn,x_ukf(:,1));


%%
t_start = 1
tic
for k =t_start:25000
    y_ukf = [x_input(k,1)-x_input(1,1) y_input(k,1)-y_input(1,1) Y(k,3:7)]';
    [CorrectedState,CorrectedStateCovariance] = correct(obj_ukf,y_ukf);
    [PredictedState,PredictedStateCovariance] = predict(obj_ukf);
    x_ukf(:,k) = obj_ukf.State(:,1);
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