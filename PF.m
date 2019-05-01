%% Particle Filter
pf = particleFilter(@stateTransitionFunc,@measurementFcn)
N = size(x,2)
%%
x_pf = zeros(size(x))
x_pf(:,1) = x(:,1)
x_pf = x_pf'
initialize(pf, 1000, x_pf(1,:),eye(8))

pf.StateEstimationMethod = 'mean'
pf.ResamplingMethod = 'systematic'

for k =t_start:K
    y_pf = [x_input(k,1)-x_input(1,1) y_input(k,1)-y_input(1,1) Y(k,3:7)]';
    x_pf(k,:) = correct(pf, y_pf);
    predict(pf);
end


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

function y_k = measurementFcn(predictedStates,measurements)
    
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
end