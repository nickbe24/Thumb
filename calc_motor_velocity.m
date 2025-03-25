function m_vel = calc_motor_velocity(m)
    
    time = m(:,1);
    position = m(:,2);
    m_vel = zeros(length(time), 2);
    m_vel(:,1) = time;

    for i = 1:length(m)-1
        m_vel(i+1,2) = (position(i+1)-position(i))/(time(i+1)-time(i));
    end


end