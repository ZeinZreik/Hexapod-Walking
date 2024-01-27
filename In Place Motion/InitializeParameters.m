global T;
global Ts;
global maxth1;global maxth2;
maxth1=30;
maxth2=50;
global start;
global accelerator;
accelerator=1;
start = 0.2;
T=2;%2sec
Ts=5;%Simulation Time

Fs = 100;
Joints_Damping = 1;
g = -9.80665;
global L0;global L1;global L2;global L3;
global q0;
L0=100; L1=53; L2=68; L3=107;%cm
q0=[pi/3 0 -pi/3 -2*pi/3 -pi 2*pi/3]*180/pi;
% L1 = 5;L2 = 10;L3 = 15;
Base_L = L0;
% Leg_th1 = 60;Leg_th2 = 0;Leg_th3 = -60;Leg_th4 = 240;Leg_th5 = 180;Leg_th6 = 120;
Mass_L1 = 0.2;Mass_L2 =0.2;Mass_L3 =0.2;

Plane = struct;
Plane.length = 2;
Plane.width = 5;
Plane.depth =0.025;%0.3 0.025
contact_k = 2500;%10000 2500
contact_b = 500;%10000 100
mu_k=0.6;
mu_s=0.8;
mu_vth=0.4;
contact_point_radius=0.0001;
raduis = 0.9;
ball=1.2;
th10 = 0;th20 = 30;th30 = -120;
thresh=0.02;