function new_vel = limit_velocity(vel,linear_limit)

    m = max(vel);
    if m>linear_limit
        ratio = linear_limit/m;
        new_vel = vel*ratio;
    
    else
        new_vel = vel;
    end



end
