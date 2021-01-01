clear
clc

% Leader
L_x = 0; 
L_y = 0;
L_v = 5;
L_accx = 0;
L_accy = 0;
LHad = 10; % degree of Leader heading angle  
L_heading = pi*LHad/180; % radians of Leader heading angle
L_ac = 15; % Leader avoidance collision 
L_c = 5;

% Follower 1
F1_x = -20;
F1_y = 21;
F1_v = 5;
F1_accx = 0;
F1_accy = 0;
heading_angle_1 = 5; % degree of follower 1 heading angle  
F1_heading = pi*heading_angle_1/180; % radians of follower 1 heading angle
F1_ac = 15;
F1_c = 5; % collision distance

% Follower 2
F2_x = -20;
F2_y = -21;
F2_v = 5;
heading_angle_2 = 4; % degree of follower 2 heading angle 
F2_heading = pi*heading_angle_2/180; % radians of follower heading angle 
F2_ac = 15;
F2_accx = 0;
F2_accy = 0;
F2_c = 5;

% time step
delta_l = 0.01;
delta_t = 0.01;

% max overshoot and rise time
mo = 0.05;
ts = 4;

% desire follower 1 distance
desire_F1_dx = -20;
desire_F1_dy = 20;
    
% desire follower 2 distance
desire_F2_dx = -20;
desire_F2_dy = -20;

i = 1;

F1_errorx_temp = 0;
F1_errory_temp = 1;
F2_errorx_temp = intmax;
F2_errory_temp = intmax;

for t=0:0.01:260
    
    % Leader make a turn
   if t > 30 && t <= 60
        L_accy = 0.2;
   elseif t > 60 && t <= 90
        L_accy = -0.2;
   elseif t > 90 && t <= 120
        L_accy = 0.2;
   elseif t > 120 && t <= 150
        L_accy = -0.2;
   elseif t > 150 && t <= 180
        L_accy = 0.2;
   elseif t > 180 && t <= 210
        L_accy = -0.1;
   else
        L_accy = 0;
   end
   
   % change desire follower position
   if t>=80&&t<140
       desire_F1_dy = -20;
       desire_F2_dy = 20;
   elseif t>=140&&t<220
       desire_F1_dy = 20;
       desire_F2_dy = -20;
   elseif t>=220
       desire_F1_dy = -20;
       desire_F2_dy = 20;
   end
   

    % Leader
    L_v = L_v + L_accx * delta_l;
    L_headingrate = L_accy / L_v;
    L_heading = L_heading + L_headingrate * delta_l;
    L_x = L_x + L_v * cos(L_heading) * delta_l;
    L_y = L_y + L_v * sin(L_heading) * delta_l;
    
    % rotate matrix
    rotate = [cos(L_heading) -sin(L_heading); sin(L_heading) cos(L_heading)];
    
    % desire Follower 1 position
    desire_F1_pos = rotate * [desire_F1_dx; desire_F1_dy];
    desire_F1_posx = L_x + desire_F1_pos(1);
    desire_F1_posy = L_y + desire_F1_pos(2);
    
    % desire Follower 2
    desire_F2_pos = rotate * [desire_F2_dx; desire_F2_dy];
    desire_F2_posx = L_x + desire_F2_pos(1);
    desire_F2_posy = L_y + desire_F2_pos(2);
        
    % 第二組參數
    % Gain scheduling: tuning the gain value
    if t>=80
    if sqrt((desire_F1_posx - F1_x)^2 + (desire_F1_posy - F1_y)^2) > 20
        mo = 0.05;
        ts = 20;
    end
    if sqrt((desire_F2_posx - F2_x)^2 + (desire_F2_posy - F2_y)^2) > 20
        mo = 0.05;
        ts = 20;
    end
    end
    
    for n = 1:10
    [F2_accx_temp, F2_accy_temp, error_x2, error_y2]=Facc(L_x, L_y, L_v, L_accx, L_accy, L_heading, L_headingrate, F2_x,F2_y, F2_v, F2_heading, desire_F2_dx,desire_F2_dy, mo,ts);
    [F1_accx_temp, F1_accy_temp, error_x, error_y]=Facc(L_x, L_y, L_v, L_accx, L_accy, L_heading, L_headingrate ,F1_x, F1_y, F1_v, F1_heading, desire_F1_dx, desire_F1_dy, mo, ts);
    
    if (F1_errorx_temp > F1_errorx_temp-error_x)
        F1_errorx_temp = error_x;
        F1_accx = F1_accx_temp;
    end
    
    if (F1_errory_temp > F1_errory_temp-error_y)
        F1_errory_temp = error_y;
        F1_accy = F1_accy_temp;
    end
   end
    
    % two followers avoidance
    P1_P2_distance = sqrt((F1_x - F2_x)^2 + (F1_y - F2_y)^2);
    if P1_P2_distance <= F1_ac + F1_c
        if F1_v >= F2_v
            F2_accx = -2;
        else
            F1_accx = -2;
        end 
    end

    % avoidance distance
    d_F2L = sqrt((L_x - F1_x)^2 + (L_y-F1_y)^2);
    
    % Follower 1
    F1_v = F1_v + F1_accx * delta_t;
    if F1_v >= 10
        F1_v = 10;
    elseif F1_v <= 2
        F1_v = 2;
    end
    
    if d_F2L < L_ac
        [F1_heading]=avoid(L_x, L_y, L_c, F1_x, F1_y, F1_v, F1_heading);
        %a = 1; % test parameter
    else
        F1_heading = F1_heading + F1_accy * delta_t / F1_v;
    end
    
    F1_x = F1_x + F1_v * cos(F1_heading) * delta_t;
    F1_y = F1_y + F1_v * sin(F1_heading) * delta_t;
    
    % Follower 2
    F2_v = F2_v + F2_accx * delta_t;
    if F2_v >= 10
        F2_v = 10;
    elseif F2_v <= 2
        F2_v = 2;
    end
    
    if d_F2L < L_ac
        [F2_heading]=avoid(L_x, L_y, L_c, F2_x, F2_y, F2_v, F2_heading);
        %b = 1; % test parameter
    else
        F2_heading = F2_heading + F2_accy * delta_t / F2_v;
    end
    F2_x = F2_x + F2_v * cos(F2_heading) * delta_t;
    F2_y = F2_y + F2_v * sin(F2_heading) * delta_t;

    
    d_xout(i) = desire_F1_posx;
    d_yout(i) = desire_F1_posy;
    d_xout2(i) = desire_F2_posx;
    d_yout2(i) = desire_F2_posy;
    L_xout(i) = L_x;
    L_yout(i) = L_y;
    F_xout(i) = F1_x;
    F_yout(i) = F1_y;
    F_xout2(i) = F2_x;
    F_yout2(i) = F2_y;
%     F_headingout(i) = F_heading*180/pi;
%     F_headingout2(i) = F_heading2*180/pi;
%     L_headingout(i) = L_heading*180/pi;
    errorxout(i) = F1_errorx_temp;
    erroryout(i) = F1_errory_temp;
    errorxout2(i) = error_x2;
    erroryout2(i) = error_y2;
%     Faccxout(i) = F_accx;
%     Faccyout(i) = F_accy;
%     Faccx2out(i) = F_accx2;
%     Faccy2out(i) = F_accy2;
%     Fvout(i) = F_v;
%     Fvout2(i) = F_v2;
%     Lvout(i) = L_v;
    
    i = i+1;
end

figure(4)
subplot(2,1,1)
plot(0:10:t,errorxout(1:1000:end),'-oblack','LineWidth',1.5,'MarkerSize',8)
hold on
plot(0:10:t,erroryout(1:1000:end),'-xred','LineWidth',1.5,'MarkerSize',8)
grid on
xlabel('時間(sec)')
ylabel('誤差(m)')
title('僚機1誤差')
legend({'x軸誤差','y軸誤差'},'FontSize',10)
subplot(2,1,2)
plot(0:10:t,errorxout2(1:1000:end),'-oblack','LineWidth',1.5,'MarkerSize',8)
hold on
plot(0:10:t,erroryout2(1:1000:end),'-xred','LineWidth',1.5,'MarkerSize',8)
grid on
xlabel('時間(sec)')
ylabel('誤差(m)')
title('僚機2誤差')
hold off

figure(5)
% pt = [1 1000 2000 3000];
pt = [1 2500 5000 7300 11000 15000 18500 22500 26000];
% pt = [7800 8350 9200 11500 12300 13200 21500 22400 23500];
hold on
grid on
plot(L_yout,L_xout,'-black','LineWidth',1,'MarkerSize',5);
plot(d_yout,d_xout,'-blue','LineWidth',1,'MarkerSize',5)
plot(d_yout2,d_xout2,'--blue','LineWidth',1,'MarkerSize',5)
plot(F_yout,F_xout,'-red','LineWidth',1,'MarkerSize',5)
plot(F_yout2,F_xout2,'--red','LineWidth',1,'MarkerSize',5)
for n = 1:1:length(pt)
    x = [L_xout(pt(n)) F_xout(pt(n)) F_xout2(pt(n)) L_xout(pt(n))];
    y = [L_yout(pt(n)) F_yout(pt(n)) F_yout2(pt(n)) L_yout(pt(n))];
    plot(y,x,'c','LineWidth',1.5,'MarkerSize',8)
    plot(y(1),x(1),'oblack','LineWidth',1.5,'MarkerSize',8)
    plot(y(2),x(2),'.m','LineWidth',1.5,'MarkerSize',12)
    plot(y(3),x(3),'xm','LineWidth',1.5,'MarkerSize',8)
%     drawnow
%     pause(1)
end
xlabel('y(m)')
ylabel('x(m)')
title('飛行軌跡')
legend({'長機','理想僚機1','理想僚機2','僚機1','僚機2','隊形','長機位置','僚機1位置','僚機2位置'},'FontSize',10)
hold off
% 
% figure(6)
% subplot(2,1,1)
% hold on
% plot(0:1:t,Faccxout(1:100:end),'-black','LineWidth',1.5,'MarkerSize',8)
% plot(0:1:t,Faccyout(1:100:end),'--red','LineWidth',1.5,'MarkerSize',8)
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機1加速度')
% legend({'x軸加速度','y軸加速度'},'FontSize',10)
% subplot(2,1,2)
% hold on
% plot(0:1:t,Faccx2out(1:100:end),'-black','LineWidth',1.5,'MarkerSize',8)
% plot(0:1:t,Faccy2out(1:100:end),'--red','LineWidth',1.5,'MarkerSize',8)
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機2加速度')
% hold off
% 
% figure(7)
% hold on
% % plot(0:1:t,Lvout(1:100:end),'-black','LineWidth',1.5,'MarkerSize',8)
% plot(0:1:t,Fvout(1:100:end),'-red','LineWidth',1.5,'MarkerSize',8)
% plot(0:1:t,Fvout2(1:100:end),'--blue','LineWidth',1.5,'MarkerSize',8)
% grid on
% xlabel('時間(sec)')
% ylabel('速度(m/s)')
% title('僚機速度')
% legend({'僚機1','僚機2'},'FontSize',10)
% hold off
% 
% figure(8)
% hold on
% % plot(0:1:t,L_headingout(1:100:end),'-black','LineWidth',1.5,'MarkerSize',8)
% plot(0:1:t,F_headingout(1:100:end),'-.red','LineWidth',1.5,'MarkerSize',8)
% plot(0:1:t,F_headingout2(1:100:end),'--blue','LineWidth',1.5,'MarkerSize',8)
% grid on
% xlabel('時間(sec)')
% ylabel('航向角(deg)')
% title('僚機航向角')
% legend({'僚機1','僚機2'},'FontSize',10)
% hold off

% figure(9)
% plot(1:1:t,L_headingrateout(1:100:end),'-black','LineWidth',1.5,'MarkerSize',8)
% grid on
% hold off

% figure(10)
% subplot(2,3,1)
% hold on
% plot(25:1:60,errorxout(2500:100:6000),'black')
% plot(25:1:60,erroryout(2500:100:6000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=30s')
% legend({'x軸誤差','y軸誤差'},'FontSize',10)
% subplot(2,3,2)
% hold on
% plot(60:1:90,errorxout(6000:100:9000),'black')
% plot(60:1:90,erroryout(6000:100:9000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=60s')
% subplot(2,3,3)
% hold on
% plot(90:1:120,errorxout(9000:100:12000),'black')
% plot(90:1:120,erroryout(9000:100:12000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=90s')
% subplot(2,3,4)
% hold on
% plot(120:1:150,errorxout(12000:100:15000),'black')
% plot(120:1:150,erroryout(12000:100:15000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=120s')
% subplot(2,3,5)
% hold on
% plot(150:1:180,errorxout(15000:100:18000),'black')
% plot(150:1:180,erroryout(15000:100:18000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=150s')
% subplot(2,3,6)
% hold on
% plot(180:1:210,errorxout(18000:100:21000),'black')
% plot(180:1:210,erroryout(18000:100:21000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=180s')
% hold off
% 
% figure(11)
% subplot(2,3,1)
% hold on
% plot(30:1:60,errorxout2(3000:100:6000),'black')
% plot(30:1:60,erroryout2(3000:100:6000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=30s')
% legend({'x軸誤差','y軸誤差'},'FontSize',10)
% subplot(2,3,2)
% hold on
% plot(60:1:90,errorxout2(6000:100:9000),'black')
% plot(60:1:90,erroryout2(6000:100:9000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=60s')
% subplot(2,3,3)
% hold on
% plot(90:1:120,errorxout2(9000:100:12000),'black')
% plot(90:1:120,erroryout2(9000:100:12000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=90s')
% subplot(2,3,4)
% hold on
% plot(120:1:150,errorxout2(12000:100:15000),'black')
% plot(120:1:150,erroryout2(12000:100:15000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=120s')
% subplot(2,3,5)
% hold on
% plot(150:1:180,errorxout2(15000:100:18000),'black')
% plot(150:1:180,erroryout2(15000:100:18000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=150s')
% subplot(2,3,6)
% hold on
% plot(180:1:210,errorxout2(18000:100:21000),'black')
% plot(180:1:210,erroryout2(18000:100:21000),'red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('t=180s')
% hold off

% figure(12)
% subplot(3,1,1)
% hold on
% plot(70:1:130,errorxout(7000:100:13000),'-black')
% plot(70:1:130,erroryout(7000:100:13000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('僚機1在t=70s~130s誤差')
% legend({'x軸誤差','y軸誤差'},'FontSize',10)
% subplot(3,1,2)
% hold on
% plot(140:1:200,errorxout(14000:100:20000),'-black')
% plot(140:1:200,erroryout(14000:100:20000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('僚機1在t=140s~200s誤差')
% subplot(3,1,3)
% hold on
% plot(310:1:360,errorxout(31000:100:36000),'-black')
% plot(310:1:360,erroryout(31000:100:36000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('僚機1在t=310s~360s誤差')
% hold off
% 
% figure(13)
% subplot(3,1,1)
% hold on
% plot(70:1:130,errorxout2(7000:100:13000),'-black')
% plot(70:1:130,erroryout2(7000:100:13000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('僚機2在t=70s~130s誤差')
% legend({'x軸誤差','y軸誤差'},'FontSize',10)
% subplot(3,1,2)
% hold on
% plot(140:1:200,errorxout2(14000:100:20000),'-black')
% plot(140:1:200,erroryout2(14000:100:20000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('僚機2在t=140s~200s誤差')
% subplot(3,1,3)
% hold on
% plot(310:1:360,errorxout2(31000:100:36000),'-black')
% plot(310:1:360,erroryout2(31000:100:36000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('誤差(m)')
% title('僚機2在t=310s~360s誤差')
% hold off
% 
% figure(15)
% subplot(3,1,1)
% hold on
% plot(70:1:130,Fvout(7000:100:13000),'-red')
% plot(70:1:130,Fvout2(7000:100:13000),'--blue')
% grid on
% xlabel('時間(sec)')
% ylabel('速度(m/s)')
% title('僚機1在t=70s~130s速度')
% legend({'僚機1','僚機2'},'FontSize',10)
% subplot(3,1,2)
% hold on
% plot(140:1:200,Fvout(14000:100:20000),'-red')
% plot(140:1:200,Fvout2(14000:100:20000),'--blue')
% grid on
% xlabel('時間(sec)')
% ylabel('速度(m/s)')
% title('僚機1在t=140s~200s速度')
% subplot(3,1,3)
% hold on
% plot(310:1:370,Fvout(31000:100:37000),'-red')
% plot(310:1:370,Fvout2(31000:100:37000),'--blue')
% grid on
% xlabel('時間(sec)')
% ylabel('速度(m/s)')
% title('僚機1在t=310s~370s速度')
% hold off
% 
% figure(18)
% subplot(3,1,1)
% hold on
% plot(70:1:130,Faccxout(7000:100:13000),'-black')
% plot(70:1:130,Faccyout(7000:100:13000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機1在t=70s~130s加速度')
% legend({'x軸加速度','y軸加速度'},'FontSize',10)
% subplot(3,1,2)
% hold on
% plot(140:1:200,Faccxout(14000:100:20000),'-black')
% plot(140:1:200,Faccyout(14000:100:20000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機1在t=140s~200s加速度')
% subplot(3,1,3)
% hold on
% plot(310:1:360,Faccxout(31000:100:36000),'-black')
% plot(310:1:360,Faccyout(31000:100:36000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機1在t=310s~360s加速度')
% hold off
% 
% figure(19)
% subplot(3,1,1)
% hold on
% plot(70:1:130,Faccx2out(7000:100:13000),'-black')
% plot(70:1:130,Faccy2out(7000:100:13000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機2在t=70s~130s加速度')
% legend({'x軸加速度','y軸加速度'},'FontSize',10)
% subplot(3,1,2)
% hold on
% plot(140:1:200,Faccx2out(14000:100:20000),'-black')
% plot(140:1:200,Faccy2out(14000:100:20000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機2在t=140s~200s加速度')
% subplot(3,1,3)
% hold on
% plot(310:1:360,Faccx2out(31000:100:36000),'-black')
% plot(310:1:360,Faccy2out(31000:100:36000),'--red')
% grid on
% xlabel('時間(sec)')
% ylabel('加速度(m/s^2)')
% title('僚機2在t=290s~340s加速度')
% hold off