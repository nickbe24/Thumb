function [pos,angles] = FK_4DOF(q1,m2,m3,m4)
    
    % Dimensions
    % q == O'
    % motor starting positions
    OA = [0; 10; 10]; 
    Om2 = [0; 14.5; -14.836]; 
    Om3 = [0; -0.5; -15.115];
    Om4 = [0; -15.5; -25.195];
    % rod end lengths
    l2 = 25.24;
    l3 = 25.24;
    l4 = 25.24;
    %straight link lengths
    l5 = 50.559;
    l6 = 54.5894;
    l7 = 40.6602;
    % bell crank 1
    AB = [0; -13; 0];
    AC = [0; -8; 5];
    %bell crank 2
    OE = [0; -14; 0];
    OF = [0; -10; 6];
    %bell crank 3
    KG = [0;-7; 0];
    KH = [0;-6; 6];
    %other links
    OK = [0; 5; 60];
    AK = OK - OA;
    KD = [0; 4.5; 5]; 
    KJ = [0; 0; 45]; 
    JI = [0; 5.5; 0];
    JZ = [0; 0; 30];  
    
    d1 = OK; d2 = KJ; d3 = JZ;
    
    % q1 is direct drive 
    
    % m2 to q2
    
    a = OA;
    P2 = Om2;
    M2 = [0; 0; m2];
    
    U2 = P2 + M2;
    S2 = (norm(U2)^2 + norm(a)^2 - l2^2)/2;
    A2 = U2(1)*a(2)*sin(q1) - U2(2)*a(3) + U2(3)*a(2)*cos(q1);
    B2 = U2(1)*a(3)*sin(q1) + U2(2)*a(2) + U2(3)*a(3)*cos(q1);
    
    q2 = atan2(A2, B2) + acos(S2 / norm([A2, B2]));
    if q2 > deg2rad(30) || q2 < -deg2rad(120)
        q2 = atan2(A2, B2) - acos(S2 / norm([A2, B2]));
    end
    
    % m3 to beta
    
    b = AB;
    P3 = Om3;
    M3 = [0; 0; m3];
    
    R2 = [cos(q1) sin(q1)*sin(q2) sin(q1)*cos(q2);
        0         cos(q2)         -sin(q2);
        -sin(q1)  cos(q1)*sin(q2) cos(q1)*cos(q2)];

    U_beta = P3+M3-R2*a;
    
    S_beta = (norm(U_beta)^2 + norm(b)^2 - l3^2)/2;
    A_beta = U_beta(1)*b(2)*sin(q1) - U_beta(2)*b(3) + U_beta(3)*b(2)*cos(q1);
    B_beta = U_beta(1)*b(3)*sin(q1) + U_beta(2)*b(2) + U_beta(3)*b(3)*cos(q1);
    
    beta = atan2(A_beta, B_beta) + acos(S_beta / norm([A_beta, B_beta])) ;
    if beta > deg2rad(90) || beta < -deg2rad(45)
        beta = atan2(A_beta, B_beta) - acos(S_beta / norm([A_beta, B_beta]));
    end
    
    
    % m4 to beta2
    u4 = OE;
    P4 = Om4;
    M4 = [0;0;m4];
    
    U_beta2 = P4 + M4;
    S_beta2 = (norm(U_beta2)^2 + norm(u4)^2 - l4^2)/2;
    A_beta2 = U_beta2(1)*u4(2)*sin(q1) - U_beta2(2)*u4(3) + U_beta2(3)*u4(2)*cos(q1);
    B_beta2 = U_beta2(1)*u4(3)*sin(q1) + U_beta2(2)*u4(2) + U_beta2(3)*u4(3)*cos(q1);
    
    beta2 = atan2(A_beta2, B_beta2) + acos(S_beta2 / norm([A_beta2, B_beta2]));
    if beta2 > deg2rad(30) || beta2 < -deg2rad(120)
        beta2 = atan2(A_beta2, B_beta2) - acos(S_beta2 / norm([A_beta2, B_beta2]));
    end
    
    % B to q3
    u1 = AC;
    u2 = AK;
    u3 = KD;
    R_beta = [1, 0, 0;
                0, cos(beta-q2), -sin(beta-q2); 
                0, sin(beta-q2), cos(beta-q2)];
    V3 = u2 - R_beta*u1;
    S3 = (l5^2 - norm(V3)^2 - norm(u3)^2) /2;
    A3 = V3(3)*u3(2) - V3(2)*u3(3);
    B3 = V3(2)*u3(2) + V3(3)*u3(3);
    
    q3 = atan2(A3, B3) + acos(S3 / norm([A3, B3]));
    if q3 > deg2rad(30) || q3 < -deg2rad(120)
        q3 = atan2(A3, B3) - acos(S3 / norm([A3, B3]));
    end
    
    % q2 and beta2 to beta3
    
    u5 = OF;
    u6 = OK;
    u7 = KG;
    R_beta_2 = [1 0 0; 0 cos(beta2) -sin(beta2); 0 sin(beta2) cos(beta2)];
    R_q2 = [1 0 0; 0 cos(q2) -sin(q2); 0 sin(q2) cos(q2)];
    
    U_beta3 = R_q2*u6-R_beta_2*u5;
    S_beta3 = (l6^2-norm(U_beta3)^2-norm(u7)^2)/2;
    A_beta3 = U_beta3(3)*u7(2) - U_beta3(2)*u7(3);
    B_beta3 = U_beta3(2)*u7(2) + U_beta3(3)*u7(3);
    
    beta3 = atan2(A_beta3, B_beta3) + acos(S_beta3 / norm([A_beta3, B_beta3]));
    if beta3 > deg2rad(90) || beta3 < -deg2rad(120)
        beta3 = atan2(A_beta3, B_beta3) - acos(S_beta3 / norm([A_beta3, B_beta3]));
    end
    
    % beta_3 and q3 to q4
    
    u8 = KH;
    u9 = KJ;
    u10 = JI;
    R_beta_3 = [1 0 0; 0 cos(beta3) -sin(beta3); 0 sin(beta3) cos(beta3)];
    R_q3 = [1 0 0; 0 cos(q2+q3) -sin(q2+q3);0 sin(q2+q3) cos(q2+q3)];
    
    U4 = R_q3*u9-R_beta_3*u8;
    S4 = (l7^2-norm(U4)^2-norm(u10)^2)/2;
    A4 = U4(3)*u10(2)-U4(2)*u10(3);
    B4 = U4(2)*u10(2)+U4(3)*u10(3);
    
    q4 = atan2(A4,B4) + acos(S4/norm([A4,B4]))-q3-q2;
    
    if q4 > deg2rad(30) || q4 < -deg2rad(120)
        q4 = atan2(A4, B4) - acos(S4 / norm([A4, B4]))-q3-q2;
    end
    
    
    % Finger Tip Configuration
    
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
    Position = T(1:3, 4)';
    global_q4 = q2+q3+q4;
    pos=[Position, global_q4];

    angles = [q1,q2,q3,q4,beta,beta2,beta3];


end

