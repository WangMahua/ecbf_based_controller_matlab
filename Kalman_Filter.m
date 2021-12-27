clear all
clc

%% Define system parameters
rng(10, 'twister')
delta_t = 0.1;
t = (0 : delta_t : 10)';
u = sin(t/10);
A = [1         0         0         delta_t   0         0         delta_t/2 0         0;
     0         1         0         0         delta_t   0         0         delta_t/2 0;
     0         0         1         0         0         delta_t   0         0         delta_t/2;
     0         0         0         1         0         0         delta_t   0         0;
     0         0         0         0         1         0         0         delta_t   0;
     0         0         0         0         0         1         0         0         delta_t;
     0         0         0         0         0         0         1         0         0;
     0         0         0         0         0         0         0         1         0;
     0         0         0         0         0         0         0         0         1;]; % 狀態轉移矩陣
B = [0;
     0;
     0;
     0;
     0;
     0;
     1;
     1;
     1;]; % 控制矩陣
C = [1 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0;]; % 觀察矩陣
D = 0; % 不討論 Y 的控制矩陣

%% System
System = ss(A,B,C,D,-1);

%% True state
x0 =[0 0 0 0 0 0 0 0 0];
[Y_True, Time_Step, X_True] = lsim(System,u);

%%  Create the measurement
R = 5;
R_Measurement = sqrt(R)*randn(length(t),1); % R = 測量noise
Y_Measurement = lsim(System,u) + R_Measurement;

%% Initialization
P = eye(9);
X_KF(:,1) = [0;
             0;
             0;
             0;
             0;
             0;
             0;
             0;
             0;];

Y_KF = zeros(length(t),3);

%% Kalman filter iteration
for i=(1:length(t)-1)
    % Prediction
    X_KF(:,i+1)= A*X_KF(:,i);
    
    Q = 5;
    Q_Prediction = sqrt(Q)*randn(length(t),1); % Q = 預測noise
    P = A*P*A' + abs(eye(9)*Q_Prediction(i));
    
    K = P*C'/(C*P*C'+abs(R_Measurement(i+1)));
    
    X_KF(:,i+1) = X_KF(:,i+1) + K*(Y_Measurement(i+1,:)'-C*X_KF(:,i+1));
    
    P = (eye(9)-K*C)*P;   

    Y_KF(i+1,:) = C*X_KF(:,i+1);
end

%% Cumulative error
Y_Measurement_Cumulative  = [0];
Y_KF_Cumulative = [0];
for i=(1:length(Y_True))
    Y_Measurement_Cumulative(1,i+1) = abs(Y_True(i,1)-Y_Measurement(i,1)) + Y_Measurement_Cumulative(1,i);
    Y_KF_Cumulative(1,i+1) = abs(Y_True(i,1)-Y_KF(i,1)) + Y_KF_Cumulative(1,i);
end

%% Generate plots
subplot(311), plot(t, X_KF(1,1:length(t)), 'r', t, X_KF(4,1:length(t)), 'b', t,X_KF(7,1:length(t)), 'm',...
                   t, X_True(1:length(t),1), 'r--', t, X_True(1:length(t),4), 'b--', t, X_True(1:length(t),7), 'm--', 'LineWidth',1.5);
title('Response with time-varying Kalman filter','FontSize', 16);
xlabel('No. of samples'), ylabel('State')
legend ('Estimated pos.','Estimated Vel.','Estimated Acc.',...
        'True pos.','True Vel.','True Acc.','FontSize', 16);
ax = gca;
ax.XAxis.FontSize = 16;
ax.YAxis.FontSize = 16;

subplot(312), plot(t,Y_True(1:length(t),1)-Y_Measurement(1:length(t),1),'b',t,Y_True(1:length(t),1)-Y_KF(1:length(t),1),'r--', 'LineWidth',2),
xlabel('No. of samples'), ylabel('Error')
legend ('Measurement Error', 'KF Error','FontSize', 16);
ax = gca;
ax.XAxis.FontSize = 16;
ax.YAxis.FontSize = 16;

subplot(313), plot(t,Y_Measurement_Cumulative(1,1:length(t)),'b',t,Y_KF_Cumulative(1,1:length(t)),'r--', 'LineWidth',2),
xlabel('No. of samples'), ylabel('Cumulative Error')
legend ('Measurement Error', 'KF Error','FontSize', 16);
ax = gca;
ax.XAxis.FontSize = 16;
ax.YAxis.FontSize = 16;