function [m1, m2, m3, m4, ang] = joint_control_4DOF(ang_s,ang_e, time, dt)

    l = time/dt;

    m = zeros(l+1,5);
    m(:,1) = 0:dt:time;
    ang = zeros(l+1,7);
    pos = zeros(l+1,4);
    
    if ang_e(1)-ang_s(1) == 0
        ang(:,1) = ang_e(1)*ones(l+1,1);
    else
        da1 = (ang_e(1)-ang_s(1))/l;
        ang(:,1) = ang_s(1):da1:ang_e(1);
    end

    if ang_e(2)-ang_s(2) == 0
        ang(:,2) = ang_e(2)*ones(l+1,1);
    else
        da2 = (ang_e(2)-ang_s(2))/l;
        ang(:,2) = ang_s(2):da2:ang_e(2);
    end

    if ang_e(3)-ang_s(3) == 0
        ang(:,3) = ang_e(3)*ones(l+1,1);
    else
        da3 = (ang_e(3)-ang_s(3))/l;
        ang(:,3) = ang_s(3):da3:ang_e(3);
    end
    if ang_e(4)-ang_s(4) == 0
        ang(:,4) = ang_e(4)*ones(l+1,1);
    else
        da3 = (ang_e(4)-ang_s(4))/l;
        ang(:,4) = ang_s(4):da3:ang_e(4);
    end
    
    
    for i=1:time/dt+1
    
        [m(i,2:5), ang(i,:)] = angles2motor_4DOF(ang(i,1),ang(i,2),ang(i,3),ang(i,4));
    
    end

    m1 = [m(:,1) m(:,2)];
    m2 = [m(:,1) m(:,3)];
    m3 = [m(:,1) m(:,4)];
    m4 = [m(:,1) m(:,5)];

end
