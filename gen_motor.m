function [shortm, m1, m2, m3, m4, actual_end_pos, ang, mv] = gen_motor(start_pos, end_pos, dt, time, error_goal_lin,error_goal_angle)

    [motor_s,ang_s] = IK_4DOF(start_pos(1),start_pos(2),start_pos(3),start_pos(4));
    
    max_time = 10;
    
    m = zeros((max_time/dt)+1,5);
    m(:,1) = 0:dt:max_time;
    ang = zeros((max_time/dt),7);
    pos = zeros((max_time/dt),4);
    mv = zeros((max_time/dt),4);
    pos(1,:) = start_pos;
    m(1,2:5) = motor_s;
    ang(1,:) = ang_s;
    
    i = 1;
    errorx = 100;
    errory = 100;
    errorz = 100;
    errorphi = 100;
    distance = norm(end_pos(1:3) - start_pos(1:3));
    
    ang_dist = end_pos(4)-start_pos(4);
    
    vel_scalar_lin = distance/time;
    vel_scalar_ang = abs(ang_dist/time);

    if vel_scalar_ang > 1
        vel_scalar_ang = .2;
    end
    
    
    
    while errorx>error_goal_lin || errory>error_goal_lin || errorz>error_goal_lin || errorphi>error_goal_angle && i<max_time/dt
        J = find_J_4DOF(ang(i,1),ang(i,2),ang(i,3),ang(i,4),ang(i,5),ang(i,6),ang(i,7),m(i,3),m(i,4),m(i,5));
        pos_diff = end_pos-pos(i,:); 
        errorx = abs(pos_diff(1));
        errory = abs(pos_diff(2));
        errorz = abs(pos_diff(3));
        errorphi = abs(pos_diff(4));
        vels = calc_velocity(pos_diff,vel_scalar_lin,vel_scalar_ang); 
        md = pinv(J)*vels'; 
        md = limit_velocity(md,8); %how to scale q1 limit
    
        m(i+1,2:5) = m(i,2:5)+dt*md'; 
        [next_pos, next_ang] = FK_4DOF(m(i+1,2),m(i+1,3),m(i+1,4),m(i+1,5));
        ang(i+1,:) = next_ang;
        pos(i+1,:) = next_pos;
        mv(i+1,:) = md;
        i=i+1;
    end
    
    shortm = cutRows(m(:,2:5));
    mv = cutRows(mv); 
    m1 = [m(1:size(shortm,1),1) shortm(:,1)];
    m2 = [m(1:size(shortm,1),1) shortm(:,2)];
    m3 = [m(1:size(shortm,1),1) shortm(:,3)];
    m4 = [m(1:size(shortm,1),1) shortm(:,4)];
    ang = fillRows(ang);
    actual_end_pos = next_pos;
end

