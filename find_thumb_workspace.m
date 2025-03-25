function [Positions, boundary, distances] = find_thumb_workspace(q1_min, q1_max,q2_min, q2_max, q3_min, q3_max, q4_min, q4_max, dq, d1 ,d2 ,d3)
    
    Positions = [];
    distances = [];
    
  
    for q1 = q1_min:dq:q1_max
        if q1 == q1_min || q1 == q1_max % if the abduction angle is min/max it is a side surface, must calculate the full workspace
            for q2 = q2_min:-dq*2:q2_max
                for q3 = q3_min:-dq*2:q3_max
                    for q4 = q4_min:-dq*2:q4_max
                        position = angles2Position_thumb(q1, q2, q3, q4, d1, d2, d3);
                        Positions(:, end+1) = position;
                        distances(end+1) = norm(position);
                    end
                end
            end
        end
        for q2 = q2_min:-dq:q2_max %else it is a middle strip and only the outside surfaces need to be calculated
            q3 = q3_min;
            q4 = q4_min;
            position = angles2Position_thumb(q1, q2, q3, q4, d1, d2, d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
            q3 = q3_max;
            q4 = q4_max;
            position = angles2Position_thumb(q1, q2, q3, q4, d1, d2, d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
        end
        for q3 = q3_min:-dq: q3_max
            q2 = q2_max;
            q4 = q4_min;
            position = angles2Position_thumb(q1, q2, q3, q4, d1, d2, d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
            q2 = q2_min;
            q4 = q4_max;
            position = angles2Position_thumb(q1, q2, q3, q4, d1, d2, d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
        end
        for q4 = q4_min:-dq:q4_max
            q2 = q2_max;
            q3 = q3_max;
            position = angles2Position_thumb(q1, q2, q3, q4, d1, d2, d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
            q2 = q2_min;
            q3 = q3_min;
            position = angles2Position_thumb(q1, q2, q3, q4, d1, d2, d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
        end

    end
    %boundary locations
    q1_sequence = [q1_min, q1_min, q1_min, q1_min, ... % q2 q3 q4 all go to max 
                   q1_min, q1_min, q1_min, ... % q2 q3 q4 all go to min 
                   q1_max, ... % q1 goes to max
                   q1_max, q1_max, q1_max,... % q2 q3 q4 all go to max
                   q1_max, q1_max, q1_max,... % q2 q3 q4 all go to min
                   q1_max, q1_min, q1_min, q1_max, q1_max, q1_min]; %cross lines

    q2_sequence = [q2_min, q2_max, q2_max, q2_max, ...
                   q2_min, q2_min, q2_min, ...
                   q2_min, ...
                   q2_max, q2_max, q2_max, ...
                   q2_min, q2_min, q2_min,...
                   q2_min, q2_min, q2_min, q2_min, q2_max, q2_max];

    q3_sequence = [q3_min, q3_min, q3_max, q3_max, ...
                   q3_max, q3_min, q3_min, ...
                   q3_min, ...
                   q3_min, q3_max, q3_max,...
                   q3_max, q3_min, q3_min,...
                   q3_min, q3_min, q3_max, q3_max, q3_max, q3_max ];

    q4_sequence = [q4_min, q4_min, q4_min, q4_max,...
                   q4_max, q4_max, q4_min, ...
                   q4_min, ...
                   q4_min, q4_min, q4_max,...
                   q4_max, q4_max, q4_min,...
                   q4_max, q4_max, q4_max, q4_max, q4_max, q4_max];

    boundary = [];
    N = 200; %interpolate 200 points between each data point for smoothness
    for i=1:length(q1_sequence)-1
        Q1 = linspace(q1_sequence(i), q1_sequence(i+1), N);
        Q2 = linspace(q2_sequence(i), q2_sequence(i+1), N);
        Q3 = linspace(q3_sequence(i), q3_sequence(i+1), N);
        Q4 = linspace(q4_sequence(i), q4_sequence(i+1), N);
        for j=1:length(Q1)
            q1 = Q1(j);
            q2 = Q2(j);
            q3 = Q3(j);
            q4 = Q4(j);
            boundary(:, end+1) = angles2Position_thumb(q1, q2, q3, q4, d1,d2,d3);
        end
    end


end