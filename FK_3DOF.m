function [Position, angles, J] = FK_3DOF(motor_positions)

qa = [0; 10; 10];
qm2 = [0; 14.5; -14.836];
qm3 = [0; -.5; -15.111];
qk = [0; 5; 40];
ab = [0; -13; 0];
ac = [0; -8; 5];
kf = [0; -6; 0];
kd = [0; 4.5; 5];
kh = [0; 0; 30];
hg = [0; 4.3; 0];
ak = qk-qa;
ht = [0; 0; 25]; % end effector

% rod end lengths
l2 = 25.24;
l3 = 25.24;
%straight link lengths
l4 = 30.92;
l5 = 31.72;

d1 = qk; d2 = kh; d3 = ht;

m1 = motor_positions(1);
m2 = motor_positions(2);
m3 = motor_positions(3);

%% Solve q1

q1 = m1;

%% Solve q2

a = qa;
P2 = qm2;
M2 = [0; 0; m2];

U2 = P2 + M2;
S2 = (norm(U2)^2 + norm(a)^2 - l2^2)/2;
A2 = U2(1)*a(2)*sin(q1) - U2(2)*a(3) + U2(3)*a(2)*cos(q1);
B2 = U2(1)*a(3)*sin(q1) + U2(2)*a(2) + U2(3)*a(3)*cos(q1);

q2 = atan2(A2, B2) + acos(S2 / norm([A2, B2]));
if q2 > deg2rad(30) || q2 < -deg2rad(120)
    q2 = atan2(A2, B2) - acos(S2 / norm([A2, B2]));
end

%% Solve Bell-crank angle

b = ab;
P3 = qm3;
M3 = [0; 0; m3];

R2 = [cos(q1) sin(q1)*sin(q2) sin(q1)*cos(q2);
    0         cos(q2)         -sin(q2);
    -sin(q1)  cos(q1)*sin(q2) cos(q1)*cos(q2)];
U_beta = P3+M3-R2*a;

S_beta = (norm(U_beta)^2 + norm(b)^2 - l3^2)/2;
A_beta = U_beta(1)*b(2)*sin(q1) - U_beta(2)*b(3) + U_beta(3)*b(2)*cos(q1);
B_beta = U_beta(1)*b(3)*sin(q1) + U_beta(2)*b(2) + U_beta(3)*b(3)*cos(q1);

beta = atan2(A_beta, B_beta) + acos(S_beta / norm([A_beta, B_beta]));
if beta > deg2rad(90) || beta < -deg2rad(45)
    beta = atan2(A_beta, B_beta) - acos(S_beta / norm([A_beta, B_beta]));
end

%% Solve q3

u1 = ac;
u2 = ak;
u3 = kd;

R_beta_3 = [1, 0, 0;
            0, cos(beta - q2) -sin(beta - q2); 
            0, sin(beta - q2) cos(beta - q2)];
V3 = u2 - R_beta_3*u1;
S3 = (l4^2 - norm(V3)^2 - norm(u3)^2) /2;
A3 = V3(3)*u3(2) - V3(2)*u3(3);
B3 = V3(2)*u3(2) + V3(3)*u3(3);

q3 = atan2(A3, B3) + acos(S3 / norm([A3, B3]));
if q3 > deg2rad(30) || q3 < -deg2rad(120)
    q3 = atan2(A3, B3) - acos(S3 / norm([A3, B3]));
end

%% Solve q4

v1 = kf;
v2 = kh;
v3 = hg;

R3 = [1, 0, 0;
      0, cos(q3) -sin(q3); 
      0, sin(q3) cos(q3)];
U4 = R3*v2 - v1;
S4 = (l5^2 - norm(U4)^2 - norm(v3)^2)/2;
A4 = U4(3)*v3(2) - U4(2)*v3(3);
B4 = U4(2)*v3(2) + U4(3)*v3(3);

q4 = atan2(A4, B4) + acos(S4 / norm([A4, B4])) - q3;
if q4 > deg2rad(30) || q4 < -deg2rad(120)
    q4 = atan2(A4, B4) - acos(S4 / norm([A4, B4])) - q3;
end

%% Finger Tip Configuration TF

T1 = [cos(q1), 0, sin(q1), 0;
      0, 1, 0, 0;
      -sin(q1), 0, cos(q1), 0;
      0, 0, 0, 1];

T2 = [1, 0, 0, 0;
      0, cos(q2), -sin(q2), 0;
      0, sin(q2), cos(q2), 0;
      0, 0, 0, 1];

T3 = [1, 0, 0, 0;
      0, cos(q3), -sin(q3), d1(2);
      0, sin(q3), cos(q3), d1(3);
      0, 0, 0, 1];

T4 = [1, 0, 0, 0;
      0, cos(q4), -sin(q4), d2(2);
      0, sin(q4), cos(q4), d2(3);
      0, 0, 0, 1];

T5 = [1, 0, 0, 0;
      0, 1, 0, d3(2);
      0, 0, 1, d3(3);
      0, 0, 0, 1];

T = T1*T2*T3*T4*T5;

angles = [q1, q2, q3, q4, beta];
Position = T(1:3, 4);

%% velocity relations

dT1_dq1 = [-sin(q1), 0, cos(q1), 0;
          0, 0, 0, 0;
          -cos(q1), 0, -sin(q1), 0;
          0, 0, 0, 0];

dT2_dq2 = [0, 0, 0, 0;
          0, -sin(q2), -cos(q2), 0;
          0,  cos(q2), -sin(q2), 0;
          0, 0, 0, 0];

dT3_dq3 = [0, 0, 0, 0;
          0, -sin(q3), -cos(q3), 0;
          0,  cos(q3), -sin(q3), 0;
          0, 0, 0, 0];

dT4_dq4 = [0, 0, 0, 0;
          0, -sin(q4), -cos(q4), 0;
          0,  cos(q4), -sin(q4), 0;
          0, 0, 0, 0];

%% q1-m1 velocity relations

dq1_dm1 = 1;


%% q2-m1,m2 velocity relations
E2 = P2(1) - sin(q1)*sin(q2)*a(2) - sin(q1)*cos(q2)*a(3);
G2 = P2(2) - cos(q2)*a(2) + sin(q2)*a(3);
H2 = P2(3) + m2 -cos(q1)*sin(q2)*a(2) - cos(q1)*cos(q2)*a(3);
I2 = E2 * (sin(q1)*sin(q2)*a(3) - sin(q1)*cos(q2)*a(2))... 
   + G2 * (sin(q2)*a(2)+cos(q2)*a(3))...
   + H2 * (cos(q1)*sin(q2)*a(3) - cos(q1)*cos(q2)*a(2));
K2 = -E2 * (cos(q1)*sin(q2)*a(2) + cos(q1)*cos(q2)*a(3)) + ...
      H2 * (sin(q1)*sin(q2)*a(2) + sin(q1)*cos(q2)*a(3));

dq2_dm1 = - (K2/I2)*dq1_dm1;
dq2_dm2 = - H2/I2;
dq2_dm3 = 0;

%% beta-m1,m2,m3 velocity relations
E3 = P3(1) - sin(q1)*sin(q2)*a(2) - sin(q1)*cos(q2)*a(3)...
     -sin(q1)*sin(q2+beta)*b(2) - sin(q1)*cos(q2+beta)*b(3);

alpha1 = -cos(q1)*sin(q2)*a(2) - cos(q1)*cos(q2)*a(3)...
         -cos(q1)*sin(q2+beta)*b(2) - cos(q1)*cos(q2+beta)*b(3);

gamma1 = -sin(q1)*cos(q2)*a(2) + sin(q1)*sin(q2)*a(3)...
         -sin(q1)*cos(q2+beta)*b(2) + sin(q1)*sin(q2+beta)*b(3);

epislon1 = sin(q1)*b(3)*sin(q2+beta) - sin(q1)*b(2)*cos(q2+beta);

G3 = P3(2) - cos(q2)*a(2) + sin(q2)*a(3)...
           - cos(q2+beta)*b(2) + sin(q2+beta)*b(3);

gamma2 = sin(q2)*a(2) + cos(q2)*a(3) + sin(q2+beta)*b(2) + cos(q2+beta)*b(3);

epislon2 = sin(q2+beta)*b(2) + cos(q2+beta)*b(3);

H3 = P3(3) + m3 - cos(q1)*sin(q2)*a(2) - cos(q1)*cos(q2)*a(3)...
                - cos(q1)*sin(q2+beta)*b(2) - cos(q1)*cos(q2+beta)*b(3);

alpha3 = sin(q1)*sin(q2)*a(2) + sin(q1)*cos(q2)*a(3)...
        +sin(q1)*sin(q2+beta)*b(2) + sin(q1)*cos(q2+beta)*b(3);

gamma3 = cos(q1)*sin(q2)*a(3) - cos(q1)*cos(q2)*a(2)...
        +cos(q1)*sin(q2+beta)*b(3) - cos(q1)*cos(q2+beta)*b(2);

epislon3 = cos(q1)*sin(q2+beta)*b(3) - cos(q1)*cos(q2+beta)*b(2);

I3 = E3*alpha1 + H3*alpha3;
K3 = E3*gamma1 + G3*gamma2 + H3*gamma3;
L3 = E3*epislon1 + G3*epislon2 + H3*epislon3;

dbeta_dm1 = -(I3/L3)*dq1_dm1 - (K3/L3)*dq2_dm1;
dbeta_dm2 = - (K3/L3)*dq2_dm2;
dbeta_dm3 = - H3/L3;

%% q3-m1,m2,m3 velocity relations
E4 = u2(2) + cos(q3)*u3(2) - sin(q3)*u3(3) - cos(beta)*u1(2) + sin(beta)*u1(3);
G4 = u2(3) + sin(q3)*u3(2) + cos(q3)*u3(3) - sin(beta)*u1(2) - cos(beta)*u1(3);
I4 = -sin(q3)*u3(2) - cos(q3)*u3(3);
K4 = sin(beta)*u1(2) + cos(beta)*u1(3);
J4 = cos(q3)*u3(2) - sin(q3)*u3(3);
L4 =-cos(beta)*u1(2) + sin(beta)*u1(3);
omega3 = - (E4*K4+G4*L4)/(E4*I4+G4*J4);

dq3_dm1 = omega3 * dbeta_dm1;
dq3_dm2 = omega3 * dbeta_dm2;
dq3_dm3 = omega3 * dbeta_dm3;

%% q4-m1,m2,m3 velocity relations
E5 = cos(q3)*v2(2) - sin(q3)*v2(3) + cos(q3+q4)*v3(2) - sin(q3+q4)*v3(3) -v1(2);
G5 = sin(q3)*v2(2) + cos(q3)*v2(3) + sin(q3+q4)*v3(2) + cos(q3+q4)*v3(3) -v1(3);

I5 = -sin(q3)*v2(2) - cos(q3)*v2(3);
K5 = -sin(q3+q4)*v3(2) - cos(q3+q4)*v3(3);
J5 = cos(q3)*v2(2) - sin(q3)*v2(3);
L5 = cos(q3+q4)*v3(2) - sin(q3+q4)*v3(3);
omega4 = - (E5*I5 + G5*J5 + E5*K5 + G5*L5)/(E5*K5 + G5*L5);

dq4_dm1 = omega4*dq3_dm1;
dq4_dm2 = omega4*dq3_dm2;
dq4_dm3 = omega4*dq3_dm3;

Q11 = dT1_dq1 * dq1_dm1 * T2 * T3 * T4 * T5;
Q12 = T1 * dT2_dq2 * dq2_dm1 * T3 * T4 * T5;
Q13 = T1 * T2 * dT3_dq3 * dq3_dm1 * T4 * T5;
Q14 = T1 * T2 * T3 * dT4_dq4 * dq4_dm1 * T5;
dT_dm1 = Q11+Q12+Q13+Q14;

Q21 = T1 * dT2_dq2 * dq2_dm2 * T3 * T4 * T5;
Q22 = T1 * T2 * dT3_dq3 * dq3_dm2 * T4 * T5;
Q23 = T1 * T2 * T3 * dT4_dq4 * dq4_dm2 * T5;
dT_dm2 = Q21+Q22+Q23;

Q31 = T1 * T2 * dT3_dq3 * dq3_dm3 * T4 * T5;
Q32 = T1 * T2 * T3 * dT4_dq4 * dq4_dm3 * T5;
dT_dm3 = Q31+Q32;

%% Linear Velocity Jacobian matrix

J = [dT_dm1(1:3, 4), dT_dm2(1:3, 4), dT_dm3(1:3, 4)];
