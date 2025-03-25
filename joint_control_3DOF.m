function [m1, m2, m3, ang] = joint_control_3DOF(ang_s,ang_e, time, dt)

    l = time/dt;
    t = 0:dt:time;
    m1 = [t' zeros(l+1,1)];
    m2 = [t' zeros(l+1,1)];
    m3 = [t' zeros(l+1,1)];
    ang = zeros(l+1,3);
    
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

    
    
    for i=1:time/dt+1
    
        [m1(i,2),m2(i,2),m3(i,2)] = angles2motor_3DOF(ang(i,1),ang(i,2),ang(i,3));
    
    end
    m1 = round(m1,3);
    m2 = round(m2,3);
    m3 = round(m3,3);
end
