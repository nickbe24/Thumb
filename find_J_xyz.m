function J = find_J_xyz(q1,q2,q3,q4,beta1,beta2,beta3,m2,m3,m4)

    % Dimensions
    % q == O'
    % motor starting positions
    qA = [0; 10; 10]; 
    qm2 = [0; 14.5; -15]; 
    qm3 = [0; 0.5; -15.219];
    qm4 = [0; -13.5; -26.154];
    
    % bell crank 1
    AB = [0; -13; 0];
    AC = [0; -8; 5];
    %bell crank 2
    qE = [0; -12; 0];
    qF = [0; -9; 6];
    %bell crank 3
    KG = [0;-6;0];
    KH = [0;-5;5];
    %other links
    qK = [0; 5; 40];
    AK = qK - qA;
    KD = [0; 4.5; 5]; 
    KJ = [0; 0; 30]; 
    JI = [0; 4.6; 0];
    JZ = [0; 0; 29];  
    
    a = qA;
    b = AB;
    P2 = qm2;
    P3 = qm3;
    P4 = qm4;
    u1 = AC;
    u2 = AK;
    u3 = KD;
    u4 = qE;
    u5 = qF;
    u6 = qK;
    u7 = KG;
    u8 = KH;
    u9 = KJ;
    u10 = JI;
    
    d1 = qK; d2 = KJ; d3 = JZ;
    
    
    % q1 vel
    dq1_dm1=1;
    
    % q2 vels
    E2 = P2(1) - sin(q1)*sin(q2)*a(2) - sin(q1)*cos(q2)*a(3);
    G2 = P2(2) - cos(q2)*a(2) + sin(q2)*a(3);
    H2 = P2(3) + m2 -cos(q1)*sin(q2)*a(2) - cos(q1)*cos(q2)*a(3);
    I2 = E2 * (sin(q1)*sin(q2)*a(3) - sin(q1)*cos(q2)*a(2))... 
       + G2 * (sin(q2)*a(2)+cos(q2)*a(3))...
       + H2 * (cos(q1)*sin(q2)*a(3) - cos(q1)*cos(q2)*a(2));
    K2 = -E2 * (cos(q1)*sin(q2)*a(2) + cos(q1)*cos(q2)*a(3)) + ...
          H2 * (sin(q1)*sin(q2)*a(2) + sin(q1)*cos(q2)*a(3));
    
    dq2_dm1 = (-K2/I2)*dq1_dm1;
    dq2_dm2 = -H2/I2;
    
    % beta1 vels
    E3 = P3(1) - sin(q1)*sin(q2)*a(2) - sin(q1)*cos(q2)*a(3) - sin(q1)*sin(beta1)*b(2) - sin(q1)*cos(beta1)*b(3);
    G3 = P3(2) - cos(q2)*a(2) + sin(q2)*a(3) - cos(beta1)*b(2) + sin(beta1)*b(3);
    H3 = P3(3) + m3 - cos(q1)*sin(q2)*a(2) - cos(q1)*cos(q2)*a(3) - cos(q1)*sin(beta1)*b(2) - cos(q1)*cos(beta1)*b(3);

    I3 = E3*(-sin(q1)*cos(beta1)*b(2) + sin(q1)*sin(beta1)*b(3)) + ...
         G3*(sin(beta1)*b(2) + cos(beta1)*b(3)) + H3*(-cos(q1)*cos(beta1)*b(2) + cos(q1)*sin(beta1)*b(3));

    J3 = E3*(-sin(q1)*cos(q2)*a(2) + sin(q1)*sin(beta1)*a(3)) + ...
         G3*(sin(q2)*a(2) + cos(q2)*a(3)) + H3*(-cos(q1)*cos(q2)*a(2) + cos(q1)*sin(q2)*a(3));

    K3 = E3*(-cos(q1)*sin(q2)*a(2) - cos(q1)*cos(q2)*a(3) - cos(q1)*sin(beta1)*b(2) - cos(q1)*cos(beta1)*b(3)) + ...
        H3*(sin(q1)*sin(q2)*a(2) + sin(q1)*cos(q2)*a(3) + sin(q1)*sin(beta1)*b(2) + sin(q1)*cos(beta1)*b(3));

    dbeta1_dm1 = -(J3/I3)*dq2_dm1 - (K3/I3)*dq1_dm1;
    dbeta1_dm2 = -(J3/I3)*dq2_dm2;
    dbeta1_dm3 = -(H3/I3);
         
    
    % beta2 vels
    
    E4 = P4(1) - sin(q1)*sin(beta2)*u4(2) - sin(q1)*cos(beta2)*u4(3);
    G4 = P4(2) - cos(beta2)*u4(2) + sin(beta2)*u4(3);
    H4 = P4(3) + m4 - cos(q1)*sin(beta2)*u4(2) - cos(q1)*cos(beta2)*u4(3);
    
    I4 = E4*(-sin(q1)*cos(beta2)*u4(2) + sin(q1)*sin(beta2)*u4(3)) + G4*(sin(beta2)*u4(2)+cos(beta2)*u4(3))...
        + H4*(-cos(q1)*cos(beta2)*u4(2) + cos(q1)*sin(beta2)*u4(3));
    K4 = E4*(-cos(q1)*sin(beta2)*u4(2)-cos(q1)*cos(beta2)*u4(3)) + H4*(sin(q1)*sin(beta2)*u4(2) - sin(q1)*cos(beta2)*u4(3));
    
    dbeta2_dm1 = (-K4/I4) * dq1_dm1;
    dbeta2_dm4 = -H4/I4;

    % beta3  vels
    
    E6 = cos(q2)*u6(2) - sin(q2)*u6(3) + cos(beta3)*u7(2) - sin(beta3)*u7(3) - cos(beta2)*u5(2) +sin(beta2)*u5(3);
    G6 = sin(q2)*u6(2) + cos(q2)*u6(3) + sin(beta3)*u7(2) + cos(beta3)*u7(3) - sin(beta2)*u5(2) -cos(beta2)*u5(3);
    I6 = E6*(-sin(beta3)*u7(2) - cos(beta3)*u7(3)) + G6*(cos(beta3)*u7(2) - sin(beta3)*u7(3));
    J6 = E6*(-sin(q2)*u6(2) - cos(q2)*u6(3)) + G6*(cos(q2)*u6(2) - sin(q2)*u6(3));
    K6 = E6*(sin(beta2)*u5(2) + cos(beta2)*u5(3)) + G6*(-cos(beta2)*u5(2) + sin(beta2)*u5(3));
    
    dbeta3_dm1 = -(J6/I6)*dq2_dm1 - (K6/I6)*dbeta2_dm1;
    dbeta3_dm2 = -(J6/I6)*dq2_dm2;
    dbeta3_dm4 = -(K6/I6)*dbeta2_dm4;

    % q3 vels
    
    E5 = u2(2) - cos(beta1-q2)*u1(2) + sin(beta1-q2)*u1(3) + cos(q3)*u3(2) - sin(q3)*u3(3);
    G5 = u2(3) - sin(beta1-q2)*u1(2) - cos(beta1-q2)*u1(3) + sin(q3)*u3(2) + cos(q3)*u3(3);

    I5 = E5*(-sin(q3)*u3(2) - cos(q3)*u3(3)) + G5*(cos(q3)*u3(2) - sin(q3)*u3(3));
    J5 = E5*(sin(beta1-q2)*u1(2) + cos(beta1-q2)*u1(3)) + G5*(-cos(beta1-q2)*u1(2) + sin(beta1-q2)*u1(3));
    K5 = E5*(-sin(beta1-q2)*u1(2) - cos(beta1-q2)*u1(3)) + G5*(cos(beta1-q2)*u1(2) - sin(beta1-q2)*u1(3));
    
    dq3_dm1 = -(J5/I5)*dbeta1_dm1 - (K5/I5)*dq2_dm1;
    dq3_dm2 = -(J5/I5) * dbeta1_dm2 - (K5/I5)*dq2_dm2;
    dq3_dm3 = -(J5/I5) * dbeta1_dm3;
    

    % q4 vels 
    
    E7 = -cos(beta3)*u8(2) + sin(beta3)*u8(3) + cos(q2+q3)*u9(2) - sin(q2+q3)*u9(3) + cos(q2+q3+q4)*u10(2) - sin(q2+q3+q4)*u10(3);
    G7 = -sin(beta3)*u8(2) - cos(beta3)*u8(3) + sin(q2+q3)*u9(2) + cos(q2+q3)*u9(3) + sin(q2+q3+q4)*u10(2) + cos(q2+q3+q4)*u10(3);

    I7 = E7*(-sin(q2+q3+q4)*u10(2) - cos(q2+q3+q4)*u10(3)) + G7*(cos(q2+q3+q4)*u10(2) - sin(q2+q3+q4)*u10(3));
    J7 = E7*(-sin(q2+q3)*u9(2) - cos(q2+q3)*u9(3) - sin(q2+q3+q4)*u10(2) - cos(q2+q3+q4)*u10(3)) ...
       + G7*(cos(q2+q3)*u9(2) - sin(q2+q3)*u9(3) + cos(q2+q3+q4)*u10(2) - sin(q2+q3+q4)*u10(3)); 
    K7 = J7;
    M7 = E7*(sin(beta3)*u8(2)+cos(beta3)*u8(3)) + G7*(-cos(beta3)*u8(2) + sin(beta3)*u8(3));
    
    dq4_dm1 = -(J7/I7)*dq3_dm1 - (K7/I7)*dq2_dm1 - (M7/I7)*dbeta3_dm1;
    dq4_dm2 = -(J7/I7)*dq3_dm2 - (K7/I7)*dq2_dm2 - (M7/I7)*dbeta3_dm2;
    dq4_dm3 = -(J7/I7)*dq3_dm3;
    dq4_dm4 = -(M7/I7)*dbeta3_dm4;

    
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
    
    dT_dm1 = dT1_dq1*dq1_dm1*T2*T3*T4*T5 + T1*dT2_dq2*dq2_dm1*T3*T4*T5 + T1*T2*dT3_dq3*dq3_dm1*T4*T5 + T1*T2*T3*dT4_dq4*dq4_dm1*T5;
    dT_dm2 = T1*dT2_dq2*dq2_dm2*T3*T4*T5 + T1*T2*dT3_dq3*dq3_dm2*T4*T5 + T1*T2*T3*dT4_dq4*dq4_dm2*T5;
    dT_dm3 = T1*T2*dT3_dq3*dq3_dm3*T4*T5 + T1*T2*T3*dT4_dq4*dq4_dm3*T5;
    dT_dm4 = T1*T2*T3*dT4_dq4*dq4_dm4*T5;
    

    J = [dT_dm1(1:3, 4), dT_dm2(1:3, 4), dT_dm3(1:3, 4), dT_dm4(1:3,4)];

end









