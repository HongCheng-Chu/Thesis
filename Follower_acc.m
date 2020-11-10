function [F_accx, F_accy, error_x, error_y] = Facc(L_x, L_y, L_v, L_accx, L_accy, L_heading, L_headingrate, F_x, F_y, F_v, F_heading, desire_dx, desire_dy, mo, ts)

% nonlinear equation
% e_f = f(x) + b(x) u   x axis
% e_l = f(y) + b(y) u   y axis

% Longitudinal
f_x = L_accx*cos(L_heading) - L_accy*sin(L_heading)...
       - desire_dx*(L_headingrate^2)*cos(L_heading) + desire_dy*(L_headingrate^2)*sin(L_heading);
% Lateral
f_y = L_accx*sin(L_heading) + L_accy*cos(L_heading)...
       - desire_dx*(L_headingrate^2)*sin(L_heading) - desire_dy*(L_headingrate^2)*cos(L_heading);
% design
error_x = L_x - F_x + desire_dx*cos(L_heading) - desire_dy*sin(L_heading);
error_y = L_y - F_y + desire_dx*sin(L_heading) + desire_dy*cos(L_heading);

error_x_time = L_v*cos(L_heading) - F_v*cos(F_heading) - desire_dx*L_headingrate*sin(L_heading)...
                 - desire_dy*L_headingrate*cos(L_heading);
error_y_time = L_v*sin(L_heading) - F_v*sin(F_heading) + desire_dx*L_headingrate*cos(L_heading)...
                 - desire_dy*L_headingrate*sin(L_heading);

% damping ratio
zeta = sqrt(log(mo)^2 / (pi^2 + log(mo)^2));

% nature freq.
wn = 3.9/(ts*zeta);

% gain
k1 = 2*zeta*wn;
k2 = wn^2;
k3 = k1;
k4 = k2;

% Follower acc. matrix
Fam = [-cos(F_heading) -sin(F_heading);sin(F_heading) -cos(F_heading)];
F_total = [-f_x - k1*error_x_time - k2*error_x; -f_y - k3*error_y_time - k4*error_y];

% Follower acc.
F_acc = Fam*F_total;
F_accx = F_acc(1);
F_accy = F_acc(2);

% acc. restriction
if F_accx >= 2
    F_accx = 2;
end
if F_accx <= -2
    F_accx = -2;
end
if F_accy >= 2
    F_accy = 2;
end
if F_accy <= -2
    F_accy = -2;
end
end