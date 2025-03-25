
clear

possible_angles = [];
for a=0:-.0175:-3*pi/2
    if IK_4DOF_angletest(0,95,60,a) == true
        possible_angles(end+1) = a;
    end

end
possible_angles = rad2deg(possible_angles);
min_a = min(possible_angles)
max_a = max(possible_angles)