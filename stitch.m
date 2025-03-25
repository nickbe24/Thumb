function m = stitch(m_list, dt)

    l = size(m_list,2);
    total_t = 0;
    m = [];

    for i=1:l
        c = m_list{i};
        if i==l
            total_t = total_t + size(c,1);
            m = [m; c(1:size(c,1),:)];
        else
            total_t = total_t + size(c,1)-1;
            m = [m; c(1:size(c,1)-1,:)];
        end
    end

    t = 0:dt:(total_t-1)*dt;

    m(:,1) = t';

end
