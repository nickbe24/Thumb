function boolean = IK_4DOF_angletest(x,y,z,angle)
   desired_position = [x y z]; 
    
   % Dimensions
    % q == O'
    % motor starting positions
    OA = [0; 10; 10]; 
    Om2 = [0; 14.5; -14.836]; 
    Om3 = [0; 0.5; -14.996];
    Om4 = [0; -13.5; -25.235];
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
    
    d3 = norm(d3);
    d2 = norm(d2);
    d1 = norm(d1);
    
    q1 = atan2(desired_position(1),desired_position(3));
    if q1<-pi/2
        q1 = pi+q1;
    elseif q1>pi/2
        q1 = q1-pi;
    end
    global_q4 = -angle;
    
    pos2d = [cos(q1) 0 -sin(q1); 0 1 0; sin(q1) 0 cos(q1)]*desired_position';
    
    x = pos2d(2)-d3*sin(global_q4); 
    y = pos2d(3)-d3*cos(global_q4);
    
    u = (x^2+y^2-(d1^2+d2^2))/(2*d1*d2); %inverse kinematics formulae from class
    if u>1
        boolean = false;
        return
    end

    theta2 = atan2(sqrt(1-u.^2),u);
    theta4 = atan2(-sqrt(1-u.^2),u);
    alpha = atan2(y,x);
    beta2 = atan2(d2*sin(theta2),d1+d2*cos(theta2));
    beta4 = atan2(d2*sin(theta4),d1+d2*cos(theta4));
    theta1 = alpha-beta2;
    theta3 = alpha-beta4;
    elbow_d_corrector = atan2(OK(2),OK(3));
    if theta4 > pi/2-elbow_d_corrector
        q2 = -(pi/2-theta1-elbow_d_corrector);
        q3 = -elbow_d_corrector+theta2;
    else
        q2 = -(pi/2-theta3-elbow_d_corrector);
        q3 = theta4-elbow_d_corrector;
    end

    q4 = -global_q4-q3-q2;
    

    
    
    % q4 and q3 to beta3
    u8 = KH;
    u9 = KJ;
    u10 = JI;
    R_q3 = [1 0 0; 0 cos(q2+q3) -sin(q2+q3);0 sin(q2+q3) cos(q2+q3)];
    R_q4 = [1 0 0; 0 cos(q2+q3+q4) -sin(q2+q3+q4); 0 sin(q2+q3+q4) cos(q2+q3+q4)];
    
    U4 = R_q3*u9+R_q4*u10;
    S4 = (norm(U4)^2+norm(u8)^2-l7^2)/2;
    A4 = U4(3)*u8(2)-U4(2)*u8(3);
    B4 = U4(2)*u8(2)+U4(3)*u8(3);
    
    beta3 = atan2(A4,B4) + acos(S4/norm([A4,B4]));
    
    if q4 > deg2rad(30) || q4 < -deg2rad(120)
        beta3 = atan2(A4, B4) - acos(S4 / norm([A4, B4]));
    end
    if isreal(beta3) ~= true
        boolean = false;
        return;
    end

    % beta3 and q2 to beta2
    
    u5 = OF;
    u6 = OK;
    u7 = KG;
    R_beta_3 = [1 0 0; 0 cos(beta3) -sin(beta3); 0 sin(beta3) cos(beta3)];
    R_q2 = [1 0 0; 0 cos(q2) -sin(q2); 0 sin(q2) cos(q2)];
    
    U_beta3 = R_q2*u6+R_beta_3*u7;
    S_beta3 = (norm(U_beta3)^2+norm(u5)^2-l6^2)/2;
    A_beta3 = U_beta3(3)*u5(2) - U_beta3(2)*u5(3);
    B_beta3 = U_beta3(2)*u5(2) + U_beta3(3)*u5(3);
    
    beta2 = atan2(A_beta3, B_beta3) + acos(S_beta3 / norm([A_beta3, B_beta3]));
    if beta2 > deg2rad(90) || beta2 < -deg2rad(120)
        beta2 = atan2(A_beta3, B_beta3) - acos(S_beta3 / norm([A_beta3, B_beta3]));
    end
    if isreal(beta2) ~= true
        boolean = false;
        return;
    end
    %q3 to beta1
    
    u1 = AC;
    u2 = AK;
    u3 = KD;
    
    V3 = R_q2*u2+R_q3*u3;
    S3 = (norm(V3)^2+norm(u1)^2-l5^2)/2;
    A3 = u1(2)*V3(3)-u1(3)*V3(2);
    B3 = u1(2)*V3(2)+u1(3)*V3(3);
    
    beta1 = atan2(A3, B3) + acos(S3/norm([A3, B3]));
    if beta1 > deg2rad(30) || beta1 < -deg2rad(120)
        beta1 = atan2(A3, B3) - acos(S3 / norm([A3, B3]));
    end
    if isreal(beta1) ~= true
        boolean = false;
        return;
    end
    
    angles = [q1 q2 q3 q4 beta1 beta2 beta3];
    % JOINT ANGLES TO MOTOR DISPLACEMENTS
    a = OA;
    P2 = Om2;
    b = AB;
    P3 = Om3;
    u4 = OE;
    P4 = Om4;
    R_q1 = [cos(q1) 0 sin(q1); 0 1 0; -sin(q1) 0 cos(q1)];
    
    % beta2 to m4
    
    R_beta_2 = [1 0 0; 0 cos(beta2) -sin(beta2); 0 sin(beta2) cos(beta2)];
    W4 = P4 - R_q1*R_beta_2*u4;
    m4 = -W4(3)-sqrt(l4^2-W4(2)^2-W4(1)^2);
    
    % beta1 to m3
    
    R_beta_1 = [1 0 0; 0 cos(beta1) -sin(beta1); 0 sin(beta1) cos(beta1)];
    W3 = P3 - R_q1*R_beta_1*b - R_q1*R_q2*a;
    m3 = -W3(3)-sqrt(l3^2-W3(2)^2-W3(1)^2);
    
    % q2 to m2
    
    W2 = P2-R_q1*R_q2*a;
    m2 = -W2(3)-sqrt(l2^2-W2(2)^2-W2(1)^2);

    q1 = rad2deg(q1);
    
    if q1<45 && q1>-45 && m4>-6 && m4<16 && m3>-21 && m3<1 && m2>-21 && m2<1
        boolean = true;
    else
        boolean = false;
    end


end