function [Positions, boundary, distances] = find_finger_workspace(q1_min, q1_max,q2_min, q2_max, q3_min, q3_max,dq,d1,d2,d3)
    
    Positions = [];
    distances = [];
    
    for q1 = q1_min:dq:q1_max 
        if q1 == q1_min || q1 == q1_max % if the abduction angle is min/max it is a side surface, must calculate the full workspace
            for q2 = q2_min:-dq*2:q2_max
                for q3 = q3_min:-dq*2:q3_max
                    q4 = q3_to_q4(q3);
                    position = angles2Position(q1, q2, q3, q4,d1,d2,d3);
                    Positions(:, end+1) = position;
                    distances(end+1) = norm(position);
                end
            end
        end
        for q2 = q2_min:-dq:q2_max %else it is a middle strip and only the outside surfaces need to be calculated
            q3 = q3_min;
            q4 = q3_to_q4(q3);
            position = angles2Position(q1, q2, q3, q4,d1,d2,d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
            q3 = q3_max;
            q4 = q3_to_q4(q3);
            position = angles2Position(q1, q2, q3, q4,d1,d2,d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
        end
    
        for q3 = q3_min:-dq:q3_max
            q2 = q2_min;
            q4 = q3_to_q4(q3);
            position = angles2Position(q1, q2, q3, q4,d1,d2,d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
            q2 = q2_max;
            q4 = q3_to_q4(q3);
            position = angles2Position(q1, q2, q3, q4,d1,d2,d3);
            Positions(:, end+1) = position;
            distances(end+1) = norm(position);
        end
    end
    
    % for boundary
    
    q1_sequence = [q1_min, q1_min, q1_min, q1_min, q1_min, ...
                   q1_max, q1_max, q1_max, q1_max, q1_max, ...
                   q1_max, q1_min, q1_min, q1_max];
    q2_sequence = [q2_min, q2_max, q2_max, q2_min, q2_min, ...
                   q2_min, q2_max, q2_max, q2_min, q2_min...
                   q2_min, q2_min, q2_max, q2_max];
    q3_sequence = [q3_min, q3_min, q3_max, q3_max, q2_min, ...
                   q3_min, q3_min, q3_max, q3_max, q2_min,...
                   q3_max, q3_max, q3_max, q3_max];
    
    boundary = [];
    N = 200;
    for i=1:length(q1_sequence)-1
        Q1 = linspace(q1_sequence(i), q1_sequence(i+1), N);
        Q2 = linspace(q2_sequence(i), q2_sequence(i+1), N);
        Q3 = linspace(q3_sequence(i), q3_sequence(i+1), N);
        for j=1:length(Q1)
            q1 = Q1(j);
            q2 = Q2(j);
            q3 = Q3(j);
            q4 = q3_to_q4(q3);
            boundary(:, end+1) = angles2Position(q1, q2, q3, q4,d1,d2,d3);
        end
    end


end
