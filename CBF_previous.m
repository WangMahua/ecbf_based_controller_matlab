clc;clear;close all;

dt = 0.05; % 20Hz

% obstacles position & radius
Po = [4 7 2];
Ro = 2;
Po1 = [7 14 3];
Ro1 = 1.5;
Po2 = [12 20 3];
Ro2 = 1.5;
Po3 = [16.5 19 3];
Ro3 = 1;

Pu = [5 -4 3]; % UAV position
ub = [3 3 3]; % UAV velocity upper bound
lb = [-3 -3 -3]; % UAV velocity lower bound

Pt = [8 1 1]; % target position
Vt = [-0.1 0.4 0]; % target velocity

for t = 0:dt:60
    
    ts = uint16(20*t); % integer time
    
    % target path
    % when t is integer in specific interval, we modify the target's velocity
    if mod(t,1) == 0
      if t > 18 && t < 23
            Vt = Vt + [-0.07 -0.05 0];
      elseif t > 22 && t < 31
            Vt = Vt + [0.08 0.07 0];
      elseif t > 30 && t < 41
            Vt = Vt + [0.03 -0.09 0];
      elseif t > 40 && t < 47
            Vt = Vt + [0.02 0.10 0];
      elseif t > 46 && t < 51
            Vt = Vt + [-0.03 0.10 0];
      elseif t > 50 && t < 57
            Vt = Vt + [0.02 -0.15 0];
      end
    end
    
    % position record
    xtplot(ts+1) = Pt(1);
    ytplot(ts+1) = Pt(2);
    ztplot(ts+1) = Pt(3);
    xuplot(ts+1) = Pu(1);
    yuplot(ts+1) = Pu(2);
    zuplot(ts+1) = Pu(3);
    
    k = 2;
    % CBF constraints
    A = [ 1  0  0;
         -1  0  0;
          0  1  0;
          0 -1  0;
          0  0  1;
          0  0 -1;
          2*(Po(1)-Pu(1)) 2*(Po(2)-Pu(2)) 2*(Po(3)-Pu(3));
          2*(Po1(1)-Pu(1)) 2*(Po1(2)-Pu(2)) 2*(Po1(3)-Pu(3));
          2*(Po2(1)-Pu(1)) 2*(Po2(2)-Pu(2)) 2*(Po2(3)-Pu(3));
          2*(Po3(1)-Pu(1)) 2*(Po3(2)-Pu(2)) 2*(Po3(3)-Pu(3))];
     
    b = [Vt(1)+k*((Pt(1)+3)-Pu(1));
        -Vt(1)+k*(Pu(1)-(Pt(1)-3));
        Vt(2)+k*((Pt(2)+3)-Pu(2));
        -Vt(2)+k*(Pu(2)-(Pt(2)-3));
        Vt(3)+k*((Pt(3)+3)-Pu(3));
        -Vt(3)+k*(Pu(3)-(Pt(3)-3));
        k*(((Po(1)-Pu(1))^2+(Po(2)-Pu(2))^2+(Po(3)-Pu(3))^2)-(Ro)^2);
        k*(((Po1(1)-Pu(1))^2+(Po1(2)-Pu(2))^2+(Po1(3)-Pu(3))^2)-(Ro1)^2);
        k*(((Po2(1)-Pu(1))^2+(Po2(2)-Pu(2))^2+(Po2(3)-Pu(3))^2)-(Ro2)^2);
        k*(((Po3(1)-Pu(1))^2+(Po3(2)-Pu(2))^2+(Po3(3)-Pu(3))^2)-(Ro3)^2)];
    
    Aeq = [];
    beq = [];
    x0 = [0 0 0];

    
    Pt = Pt + dt*Vt; % update Target position

    % cost function
    f = @(x) (x(1)-Vt(1))^2 + (x(2)-Vt(2))^2 + (x(3)-Vt(3))^2 + ... % keeping the same velocity with the target
        (Pu(3) - 3)^2; % maitaining the horizontal distance of 3m

    
    %  solve optimal eq
    X_0 = fmincon(f,x0,A,b,Aeq,beq,lb,ub);
               
    Pu = Pu + X_0; % update UAV position
end

f1 = figure(1);
grid on;
hold on;
uav = animatedline('Color','r','Marker','o');
car = animatedline('Color','b','Marker','*');
[X,Y,Z] = cylinder(Ro);
Cyl = surf(X + Po(1), Y + Po(2), Z*6);
[X1,Y1,Z1] = cylinder(Ro1);
Cyl = surf(X1 + Po1(1), Y1 + Po1(2), Z1*6);
[X2,Y2,Z2] = cylinder(Ro2);
Cyl = surf(X2 + Po2(1), Y2 + Po2(2), Z2*6);
[X3,Y3,Z3] = cylinder(Ro3);
Cyl = surf(X3 + Po3(1), Y3 + Po3(2), Z3*6);
axis([0, 20, 0, 20]);
drawnow

% t = 1;
% while (t < 1201)
%     figure(f1);
%     addpoints(uav,xuplot(t), yuplot(t), zuplot(t));
%     addpoints(car,xtplot(t), ytplot(t), ztplot(t));
%     drawnow
%     t = t+1;
% end

% f2 = figure(2);
% t = 1;
% while (t < 1201)
%     figure(f2);
%     t = t+1;
%     plot3(xuplot(t),yuplot(t),zuplot(t)); hold on;
% end

figure(2);
plot3(xuplot,yuplot,zuplot); hold on; 
plot3(xtplot,ytplot,ztplot); hold on; 

% obstacle
[X_0,Y_0,Z_0]=sphere;
X_0 = X_0*Ro;
Y_0 = Y_0*Ro;
Z_0 = Z_0*Ro;
surf(X_0+Po(1),Y_0+Po(2),Z_0+Po(3));hold on; 

[X_1,Y_1,Z_1]=sphere;
X_1 = X_1*Ro1;
Y_1 = Y_1*Ro1;
Z_1 = Z_1*Ro1;
surf(X_1+Po1(1),Y_1+Po1(2),Z_1+Po1(3));hold on; 

[X_2,Y_2,Z_2]=sphere;
X_2 = X_2*Ro2;
Y_2 = Y_2*Ro2;
Z_2 = Z_2*Ro2;
surf(X_2+Po2(1),Y_2+Po2(2),Z_2+Po2(3));hold on; 

[X_3,Y_3,Z_3]=sphere;
X_3 = X_3*Ro3;
Y_3 = Y_3*Ro3;
Z_3 = Z_3*Ro3;
surf(X_3+Po3(1),Y_3+Po3(2),Z_3+Po3(3));hold on; 
axis equal;
