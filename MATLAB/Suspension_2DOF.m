%% 车辆小贾原创
close all
clc
%% Parameters of the Suspension
ms  = 500;% Sprung Mass (kg) 
mu = 66.7;% Unsprung Mass (kg)
ks  = 27000;% Suspension Stiffness (N/m) 
kw = 268000;% Wheel stiffness (N/m)
cs  = 1500;% Suspension Inherent Damping coefficient (sec/m)

%% State Space dot_x = A*X+Bu
A = [ 0 1 0 -1 ;
    -ks/ms -cs/ms 0 cs/ms;
      0 0 0 1; 
    ks/mu 0 -kw/mu -cs/mu];
B = [0 ; 1/ms; 0; -1/mu];
E = [0;0;-1;0];
C = [-ks/ms -cs/ms 0 cs/ms;  %ddot_Zs Zs-Zus Zus-Zr
        1      0    0   0;
        0      0    1   0];
D=[1/ms; 0; 0];
T = [ 0; 0; 0];
%在simulink模块中，改变C和D为了直接输出状态变量
sim('pid_active_suspension')

%% 优化结果
PAS_Accleration = rms(Acceleration_PAS);
ACT_Accleration = rms(Acceleration_ACT);
percent = abs(PAS_Accleration - ACT_Accleration)/PAS_Accleration;
fprintf("簧载质量加速度优化了%f%%.\n",percent*100);
%% Plot
% 簧载质量加速度
figure (1)
p=plot(Time,Acceleration_ACT,'r',Time,Acceleration_PAS,'b');
p(1).LineWidth = 1.7;
p(2).LineStyle  = '--';
p(2).LineWidth = 0.8;
grid
title ('\fontsize{17}簧载质量加速度','fontname','楷体') 
xlabel('\fontsize{20}Time(s)') 
ylabel('\fontsize{20}Acceleration (m/s2)'); 
legend({'Active','Passive'},'FontSize',16,'FontWeight','bold')
% 悬架动挠度
figure (2)
p=plot(Time,Suspension_Travel_ACT,'r',Time,Suspension_Travel_PAS,'b');
p(1).LineWidth = 1.7;
p(2).LineStyle  = '--';
p(2).LineWidth = 0.8;
grid
title ('\fontsize{17}悬架动挠度','fontname','楷体') 
xlabel('\fontsize{20}Time (s)') 
ylabel('\fontsize{20}displacement (m)'); 
legend({'Active','Passive'},'FontSize',16,'FontWeight','bold')
% 轮胎动载荷
Fact = Deflection_ACT*kw;
Fpas = Deflection_PAS*kw;
figure (3)
p=plot(Time,Fact,'r',Time,Fpas,'b');
p(1).LineWidth = 1.7;
p(2).LineStyle  = '--';
p(2).LineWidth = 0.8;
grid
title ('\fontsize{17}轮胎动载荷','fontname','楷体') 
xlabel('\fontsize{20}Time (s)') 
ylabel('\fontsize{20}Dynamic tyre loads (N)'); 
legend({'Active','Passive'},'FontSize',16,'FontWeight','bold')
% Actuator Force
figure (4)
p=plot(Time,ACTUATOR_FORCE,'r');
p(1).LineWidth = 1.3;
grid
title ('\fontsize{17}作动器产生的力','fontname','楷体') 
xlabel('\fontsize{20}Time (s)') 
ylabel('\fontsize{20}Actuator Force (N)'); 
legend({'Actuator force'},'FontSize',16,'FontWeight','bold')

