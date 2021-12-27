function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input


% process inputs
delta_t = 0.05; 

x_d = u(1:3);
b1_d = u(4:6);
v_d = [0; 0; 0];
a_d = [0; 0; 0];
% current state
x  = u(7:9);
v  = u(10:12);
R  = reshape(u(13:21),3,3);
Omega = u(22:24);
t = u(end);
Omega_hat = hat(Omega);

%parameters
m = P.mass;
g = P.gravity;
k_x = P.kx;
k_v = P.kv;
k_R = P.kR;
k_Omega = P.kOmega;
J = [P.Jxx 0 0;...
    0 P.Jyy 0;...
    0 0 P.Jzz];
e_3 = [0; 0; 1];



% trajectory tracking
e_x = x - x_d;
e_v = v - v_d;
% attitude tracking

b3_d = -(-k_x*e_x - k_v*e_v - m*g*e_3 + m*a_d)/norm(-(-k_x*e_x - k_v*e_v - m*g*e_3 + m*a_d));
b2_d = cross(b3_d, b1_d)/norm(cross(b3_d, b1_d));
R_d = [cross(b2_d, b3_d), b2_d, b3_d];
R_d_dot = (R_d - P.R_d_last)/delta_t;
Omega_d_hat = R_d'*R_d_dot;
Omega_d = vee(Omega_d_hat);
e_Omega = Omega - R'*R_d*Omega_d;

e_R = 1/2 * vee(R_d'*R - R'*R_d);


% Desired total thrust & Moment
f = dot((k_x*e_x + k_v*e_v + m*g*e_3 - m*a_d), R*e_3) ; 
M = -k_R*e_R - k_Omega*e_Omega + cross(Omega, J*Omega) - J*(Omega_hat*R'*R_d*Omega_d);

P.R_d_last = R_d;

out = [f;M];
end