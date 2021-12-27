clc;clear;close all;

dt = 0.05; % 20Hz

% obstacles position & radius
Po = [4 7 2];
Ro = 2;
V0 = [-0.1 0.4 0]; % target velocity

Po1 = [7 14 3];
Ro1 = 1.5;
V1 = [-0.1 0.4 0]; % target velocity

Po2 = [12 20 3];
Ro2 = 1.5;
V2 = [-0.1 0.4 0]; % target velocity

Po3 = [16.5 19 3];
Ro3 = 1;
V3 = [-0.1 0.4 0]; % target velocity

Pu = [5 -4 3]; % UAV position
Ru = 1;
ub = [3 3 3]; % UAV velocity upper bound
lb = [-3 -3 -3]; % UAV velocity lower bound

Pt = [8 1 1]; % target position
Vt = [-0.1 0.4 0]; % target velocity
Rt = 1;

pos_plot = zeros(6,3,1000);
% pos : self / target / obstacle0 / obstacle1 / obstacle2 /obstacle3
now_pos_plot = [Pu(1) Pu(2) Pu(3);  
                Pt(1) Pt(2) Pt(3);
                Po(1) Po(2) Po(3);
                Po1(1) Po1(2) Po1(3);
                Po2(1) Po2(2) Po2(3);
                Po3(1) Po3(2) Po3(3)];
 

            
now_vel_plot = [0 0 0;  
            Vt(1) Vt(2) Vt(3);
            0.05 0.05 0;
            0.05 0.05 0;
            0.05 0.05 0;
            0.07 0 0];
            
now_acc_plot = [0 0 0;  
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0;
            0 0 0];            
       
last_target_v = [0 0 0];
last_target_p = Pt;
        
k1 = 2;
k2 = 2;

        
for t = 0:dt:60
    ts = uint16(20*t); % integer time
    
    % target path
    % when t is integer in specific interval, we modify the target's velocity
    last_target_v = now_vel_plot(2,:);
    
    now_vel_plot=update_v(now_vel_plot,t);
    
    % position record

    pos_plot(:,:,ts+1)=now_pos_plot;
    
 
%     xtplot(ts+1) = Pt(1);
%     ytplot(ts+1) = Pt(2);
%     ztplot(ts+1) = Pt(3);
%     xuplot(ts+1) = Pu(1);
%     yuplot(ts+1) = Pu(2);
%     zuplot(ts+1) = Pu(3);

    Au = now_acc_plot(1,:);
    At = now_acc_plot(2,:);    
    Ao = now_acc_plot(3,:);
    Ao1 = now_acc_plot(4,:);
    Ao2 = now_acc_plot(5,:);
    Ao3 = now_acc_plot(6,:);   

    Vu = now_vel_plot(1,:);
    Vt = now_vel_plot(2,:);    
    Vo = now_vel_plot(3,:);
    Vo1 = now_vel_plot(4,:);
    Vo2 = now_vel_plot(5,:);
    Vo3 = now_vel_plot(6,:);    
    
    
    Pu  = now_pos_plot(1,:);
    Pt  = now_pos_plot(2,:);
    Po  = now_pos_plot(3,:);
    Po1 = now_pos_plot(4,:);
    Po2 = now_pos_plot(5,:);
    Po3 = now_pos_plot(6,:);
    
    k = 2;
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
        k*(((Po(1)-Pu(1))^2+(Po(2)-Pu(2))^2+(Po(3)-Pu(3))^2)-(Ro)^2)+...
            2*(Vo(1)*(Po(1)-Pu(1))+Vo(2)*(Po(2)-Pu(2))+Vo(3)*(Po(3)-Pu(3)));
        k*(((Po1(1)-Pu(1))^2+(Po1(2)-Pu(2))^2+(Po1(3)-Pu(3))^2)-(Ro1)^2)+...
            2*(Vo1(1)*(Po1(1)-Pu(1))+Vo1(2)*(Po1(2)-Pu(2))+Vo1(3)*(Po1(3)-Pu(3)));
        k*(((Po2(1)-Pu(1))^2+(Po2(2)-Pu(2))^2+(Po2(3)-Pu(3))^2)-(Ro2)^2)+...
            2*(Vo2(1)*(Po2(1)-Pu(1))+Vo(2)*(Po2(2)-Pu(2))+Vo2(3)*(Po2(3)-Pu(3)));
        k*(((Po3(1)-Pu(1))^2+(Po3(2)-Pu(2))^2+(Po3(3)-Pu(3))^2)-(Ro3)^2)+...
            2*(Vo3(1)*(Po3(1)-Pu(1))+Vo3(2)*(Po3(2)-Pu(2))+Vo3(3)*(Po3(3)-Pu(3)))];
    
    Aeq = [];
    beq = [];
    x0 = [0 0 0];
    
    % cost function
    f = @(x) (x(1)-Vt(1))^2 + (x(2)-Vt(2))^2 + (x(3)-Vt(3))^2 + ... % keeping the same velocity with the target
        (Pu(3) - 3)^2; % maitaining the horizontal distance of 3m
    v_new = fmincon(f,x0,A,b,Aeq,beq,lb,ub);
 
    now_vel_plot(1,:) = v_new;
    now_pos_plot = now_pos_plot + dt*now_vel_plot;
end

% f1 = figure(1);
% grid on;
% hold on;
% uav = animatedline('Color','r','Marker','o');
% car = animatedline('Color','b','Marker','*');
% [X,Y,Z] = cylinder(Ro);
% Cyl = surf(X + Po(1), Y + Po(2), Z*6);
% [X1,Y1,Z1] = cylinder(Ro1);
% Cyl = surf(X1 + Po1(1), Y1 + Po1(2), Z1*6);
% [X2,Y2,Z2] = cylinder(Ro2);
% Cyl = surf(X2 + Po2(1), Y2 + Po2(2), Z2*6);
% [X3,Y3,Z3] = cylinder(Ro3);
% Cyl = surf(X3 + Po3(1), Y3 + Po3(2), Z3*6);
% axis([0, 20, 0, 20]);
% drawnow

% t = 1;
% while (t < 1201)
%     figure(f1);
%     addpoints(uav,xuplot(t), yuplot(t), zuplot(t));
%     addpoints(car,xtplot(t), ytplot(t), ztplot(t));
%     drawnow
%     t = t+1;
% end

f2 = figure(2);


t = 1;
mv_r = 1;
uav = animatedline('Color','r','Marker','o');
while (t < 1201)
    figure(f2);

    t = t+1;    
%     ax1 = subplot(1,2,1);
    
    scatter3(pos_plot(1,1,1:t),pos_plot(1,2,1:t),pos_plot(1,3,1:t),'filled');hold on;
    scatter3(pos_plot(2,1,1:t),pos_plot(2,2,1:t),pos_plot(2,3,1:t),'*');hold on;
    
    % draw obstacle
    [X_0,Y_0,Z_0]=sphere;
    X_0 = X_0*Ro;
    Y_0 = Y_0*Ro;
    Z_0 = Z_0*Ro;
    surf(X_0+pos_plot(3,1,t),Y_0+pos_plot(3,2,t),Z_0+pos_plot(3,3,t));hold on ;
   
    [X_1,Y_1,Z_1]=sphere;
    X_1 = X_1*Ro1;
    Y_1 = Y_1*Ro1;
    Z_1 = Z_1*Ro1;
    surf(X_1+pos_plot(4,1,t),Y_1+pos_plot(4,2,t),Z_1+pos_plot(4,3,t));hold on ;
   
    [X_2,Y_2,Z_2]=sphere;
    X_2 = X_2*Ro2;
    Y_2 = Y_2*Ro2;
    Z_2 = Z_2*Ro2;
    surf(X_2+pos_plot(5,1,t),Y_2+pos_plot(5,2,t),Z_2+pos_plot(5,3,t));hold on ;
    
    [X_3,Y_3,Z_3]=sphere;
    X_3 = X_3*Ro3;
    Y_3 = Y_3*Ro3;
    Z_3 = Z_3*Ro3;
    surf(X_3+pos_plot(6,1,t),Y_3+pos_plot(6,2,t),Z_3+pos_plot(6,3,t));hold on ;

    axis equal;
    
    
%     ax2 = subplot(1,2,2);
%     scatter3(pos_plot(1,1,1:t),pos_plot(1,2,1:t),pos_plot(1,3,1:t),'filled');hold on;
%     scatter3(pos_plot(2,1,1:t),pos_plot(2,2,1:t),pos_plot(2,3,1:t),'*');hold on;
%     
%     % draw obstacle
%     [X_01,Y_01,Z_01]=sphere;
%     X_01 = X_01*Ro;
%     Y_01 = Y_01*Ro;
%     Z_01 = Z_01*Ro;
%     surf(X_01+pos_plot(3,1,t),Y_01+pos_plot(3,2,t),Z_01+pos_plot(3,3,t));hold on ;
%    
%     [X_11,Y_11,Z_11]=sphere;
%     X_11 = X_11*Ro1;
%     Y_11 = Y_11*Ro1;
%     Z_11 = Z_11*Ro1;
%     surf(X_11+pos_plot(4,1,t),Y_11+pos_plot(4,2,t),Z_11+pos_plot(4,3,t));hold on ;
%    
%     [X_21,Y_21,Z_21]=sphere;
%     X_21 = X_21*Ro2;
%     Y_21 = Y_21*Ro2;
%     Z_21 = Z_21*Ro2;
%     surf(X_21+pos_plot(5,1,t),Y_21+pos_plot(5,2,t),Z_21+pos_plot(5,3,t));hold on ;
%     
%     [X_31,Y_31,Z_31]=sphere;
%     X_31 = X_31*Ro3;
%     Y_31 = Y_31*Ro3;
%     Z_31 = Z_31*Ro3;
%     surf(X_31+pos_plot(6,1,t),Y_31+pos_plot(6,2,t),Z_31+pos_plot(6,3,t));hold on ;
%     axis equal;

    view(0,90);
    
    pause(0.0001);
    hold off; 
end

function v_plot = update_v(v_plot,t)
    
    if mod(t,1) == 0
      if t > 18 && t < 23
            v_plot(2,:) = v_plot(2,:) + [-0.07 -0.05 0];
            v_plot(3,:) = v_plot(3,:) + [-0.05 -0.05 0];
            v_plot(4,:) = v_plot(4,:) + [-0.05 -0.05 0];
            v_plot(5,:) = v_plot(5,:) + [-0.05 -0.05 0];
            v_plot(6,:) = v_plot(6,:) + [-0.07 -0.00 0];
      elseif t > 22 && t < 31
            v_plot(2,:) = v_plot(2,:) + [0.08 0.07 0];
            v_plot(3,:) = v_plot(3,:) + [0.05 0.05 0];
            v_plot(4,:) = v_plot(4,:) + [0.05 0.05 0];
            v_plot(5,:) = v_plot(5,:) + [0.05 0.05 0];
            v_plot(6,:) = v_plot(6,:) + [0.07 0.00 0];
      elseif t > 30 && t < 41
            v_plot(2,:) = v_plot(2,:) + [0.03 -0.09 0];

            v_plot(3,:) = v_plot(3,:) + [-0.05 -0.05 0];
            v_plot(4,:) = v_plot(4,:) + [-0.05 -0.05 0];
             v_plot(5,:) = v_plot(5,:) + [-0.05 -0.05 0];
             v_plot(6,:) = v_plot(6,:) + [-0.07 -0.00 0];
      elseif t > 40 && t < 47
            v_plot(2,:) = v_plot(2,:) + [0.02 0.10 0];

            v_plot(3,:) = v_plot(3,:) + [0.05 0.05 0];
            v_plot(4,:) = v_plot(4,:) + [0.05 0.05 0];
            v_plot(5,:) = v_plot(5,:) + [0.05 0.05 0];
            v_plot(6,:) = v_plot(6,:) + [0.07 0.00 0];
      elseif t > 46 && t < 51
            v_plot(2,:) = v_plot(2,:) + [-0.03 0.10 0];

            v_plot(3,:) = v_plot(3,:) + [-0.05 -0.05 0];
            v_plot(4,:) = v_plot(4,:) + [-0.05 -0.05 0];
            v_plot(5,:) = v_plot(5,:) + [-0.05 -0.05 0];
            v_plot(6,:) = v_plot(6,:) + [-0.07 0.00 0];
      elseif t > 50 && t < 57
            v_plot(2,:) = v_plot(2,:) + [0.02 -0.15 0];

            v_plot(3,:) = v_plot(3,:) + [0.05 0.05 0];
            v_plot(4,:) = v_plot(4,:) + [0.05 0.05 0];
            v_plot(5,:) = v_plot(5,:) + [0.05 0.05 0]; 
            v_plot(6,:) = v_plot(6,:) + [0.07 0.00 0];
      end
    end
end

function b_content = input_ub(r,v,R,r_o,v_o,a_o,R_o)
    global k1;
    global k2;
b_content = k1*((r(1)-r_o(1))^2+(r(2)-r_o(2))^2+(r(3)-r_o(3))^2-(R+R_o)^2)+...
    2*k2*((r(1)-r_o(1))*(v(1)-v_o(1))+(r(2)-r_o(2))*(v(2)-v_o(2))+(r(2)-r_o(2))*(v(2)-v_o(2)))+...
    2*((v(1)-v_o(1))^2+(v(2)-v_o(2))^2+(v(3)-v_o(3))^2)-...
    2*((r(1)-r_o(1))*a_o(1)+(r(2)-r_o(2))*a_o(2)+(r(3)-r_o(3))*a_o(3));

end
