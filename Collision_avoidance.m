function [F_heading] = avoid(L_x, L_y, L_c, F_x, F_y, F_v, F_heading)
d_F2L = sqrt((L_x - F_x)^2 + (L_y - F_y)^2);
v_F2L = F_v;
d_dot_v = dot([F_v*cos(F_heading) F_v*sin(F_heading)],[L_x - F_x L_y - F_y]);
lambda = acos(d_dot_v / v_F2L / d_F2L);
if d_F2L > L_c
        lambda_plus = asin(L_c / d_F2L);
else
        lambda_plus = pi/2;
end
lambda_minus = -lambda_plus;
if d_dot_v >= 0 && d_F2L * sin(lambda) < L_c
    if abs(lambda_plus - lambda) <= abs(lambda_minus - lambda)
        F_heading = F_heading + abs(lambda_plus - lambda);
    elseif abs(lambda_plus - lambda) > abs(lambda_minus - lambda)
        F_heading = F_heading - abs(lambda_minus - lambda);
    end 
end
end