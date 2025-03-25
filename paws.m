function m_paused = paws(last_m, pause_time,dt)

    c = last_m(end,:);
    m1 = ones(pause_time/dt+1,1)*c(1);
    m2 = ones(pause_time/dt+1,1)*c(2);
    m_paused = [m1 m2];
end

