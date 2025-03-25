function vel = calc_velocity(pos_diff,vel_scalar_lin,vel_scalar_ang)

    if norm(pos_diff(1:3))== 0
        vel_lin = [0 0 0];
    else
        vel_lin = pos_diff(1:3)/norm(pos_diff(1:3)) * vel_scalar_lin;
    end

    if norm(pos_diff(4))==0
        vel_ang = 0;
    else
        vel_ang = pos_diff(4)/norm(pos_diff(4)) * vel_scalar_ang;
    end

    vel = [vel_lin vel_ang 0 0];
end
