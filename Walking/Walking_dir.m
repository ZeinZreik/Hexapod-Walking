global T;
global q1;
global q2;
step_height=30;
point_num=20;
step_length=110;
body_height=80;
direction=90;%Y dir  ,dir=0 it means X

L0=100; L1=53; L2=68; L3=107;
length_min=L0+L1;
length_max = length_min+sqrt((L2+L3)^2-body_height^2);
LL= (length_min+length_max)/2;
tic;
T=0.5;% the period of the step
dt=T/point_num;
t=0:dt:T;
dstep=step_length/point_num;
%fot the first
zeroSpeed=0;
u1i=-step_length/2;
u1f=+step_length/2;
u1_i=zeroSpeed;
u1_f=zeroSpeed;
b1=(3*(u1f-u1i)-T*u1_f)/(T^2);%the constants of the equation u1(t)
a1=(u1_i-u1_f)/(T^2)-2*(u1f-u1i)/(T^3);

%for the second
u2i=-u1i;%+step_length/2;
u2f=-u1f;%-step_length/2;
u2_i=zeroSpeed;
u2_f=zeroSpeed;
b2=(3*(u2f-u2i)-T*u2_f)/(T^2);%the constants of the equation u(t)
a2=(u2_i-u2_f)/(T^2)-2*(u2f-u2i)/(T^3);
%constants for 2nd degree poly 
u2Thalf=(u2i+u2f)/2;
h=step_height;
mu=(u2Thalf^2-(u2i+u2f)*u2Thalf+(u2i*u2f));
cu=-h*(u2i*u2f)/mu;
au=-h/mu;
bu=h*(u2i+u2f)/mu;
% 2nd for the first tri
u1Thalf = (u1i + u1f) / 2;
h1 = -step_height*1.5;%*************
mu1 = (u1Thalf^2 - (u1i + u1f) * u1Thalf + (u1i * u1f));
cu1 = -h1 * (u1i * u1f) / mu1;
au1 = -h1 / mu1;
bu1 = h1 * (u1i + u1f) / mu1;

zeroSpeed=0;
%when translation the traj is not simple(v(0)=0 v(T)=0 (stance))
%swing second degree equation
 for i=1:point_num+1
     % the position is a 3 degree polynomal in two phases
   %step1(i)=dstep*t(i)/dt-step_length/2;
   u1(i)=a1*(t(i))^3+b1*(t(i))^2+u1_i*t(i)+u1i;%position with respect to time in U Z
   % the first tripod is in stance phase
   % and the second tripod is in swing phase
   transX_tri_1(i)=u1(i)*cos(direction*pi/180);
   transY_tri_1(i)=u1(i)*sin(direction*pi/180);
   transZ_tri_1(i) = 0;
   
   u2(i)=a2*(t(i))^3+b2*(t(i))^2+u2_i*t(i)+u2i;%position with respect to time in U Z
   transX_tri_2(i)=u2(i)*cos(direction*pi/180);
   transY_tri_2(i)=u2(i)*sin(direction*pi/180);
   transZ_tri_2(i)=au*(u2(i))^2+bu*u2(i)+cu;
   %velocity
   %v1=dstep/dt;
   %vX_1=v1*cos(direction*pi/180);
   %vY_1=v1*sin(direction*pi/180);
   u1_(i)=3*a1*(t(i))^2+2*b1*(t(i))+u1_i;%speed
   vX_1(i)=u1_(i)*cos(direction*pi/180);
   vY_1(i)=u1_(i)*sin(direction*pi/180);
   vZ_1(i)=0;

   u2_(i)=3*a2*(t(i))^2+2*b2*(t(i))+u2_i;%speed
   vX_2(i)=u2_(i)*cos(direction*pi/180);
   vY_2(i)=u2_(i)*sin(direction*pi/180);
   vZ_2(i)=2*au*u2_(i)*u2(i)+bu*u2_(i);%speed Z 2
   
   %after the traj we must calc q for each tripod
   q1(i,:,:)=IKdTripod(vX_1(i),vY_1(i),vZ_1(i),0,0,0,transX_tri_1(i),transY_tri_1(i),body_height,0,0,0,LL,1,2);
   q2(i,:,:)=IKdTripod(vX_2(i),vY_2(i),vZ_2(i),0,0,0,transX_tri_2(i),transY_tri_2(i),body_height+transZ_tri_2(i),0,0,0,LL,2,2);
   q1(i+point_num,:,:)=IKdTripod(vX_2(i),vY_2(i),vZ_2(i),0,0,0,transX_tri_2(i),transY_tri_2(i),body_height+transZ_tri_2(i),0,0,0,LL,1,2);
   q2(i+point_num,:,:)=IKdTripod(vX_1(i),vY_1(i),vZ_1(i),0,0,0,transX_tri_1(i),transY_tri_1(i),body_height,0,0,0,LL,2,2);
 end
 toc;
q1(:,:,1:3)=q1(:,:,1:3);
q2(:,:,1:3)=q2(:,:,1:3);
q1(:,:,4:6)=(q1(:,:,4:6))*60/(2*pi);
q2(:,:,4:6)=(q2(:,:,4:6))*60/(2*pi);

tnew=0:dt:2*T;
trax=[transX_tri_1 transX_tri_2];
tray=[transY_tri_1 transY_tri_2]; 
traz=[zeros(1,point_num) transZ_tri_2];
plot(tray(1:end-1),traz);
xlabel('Y(mm)');
ylabel('Z(mm)'); 
%zlabel('Z(mm)'); 
grid on;
plot(tnew,traz);
xlabel('Time t(sec)'); ylabel('Z(mm)'); 
grid on;
figure();
%tnew1=T+dt:dt:2*T;
%plot(t,transY_tri_2);
%hold on;
%plot(tnew1,transY_tri_1);

%tnew1=T+dt:dt:2*T;
%plot(t,vY_1);
%hold on;
%plot(tnew1,vY_1);
%
legIDd=1;
subplot(2,3,1);
plot(tnew,q1(:,legIDd,1));  title('theta1(t)');xlabel('second');ylabel('degree');
subplot(2,3,2);
plot(tnew,q1(:,legIDd,2));  title('theta2(t)');
xlabel('second');
ylabel('degree');
subplot(2,3,3);
plot(tnew,q1(:,legIDd,3));  title('theta3(t)');
xlabel('second');
ylabel('degree');
subplot(2,3,4);
plot(tnew,q1(:,legIDd,4));  title('theta1 '' (t)');
xlabel('second');
ylabel('rpm');
subplot(2,3,5);
plot(tnew,q1(:,legIDd,5));  title('theta2 '' (t)');
xlabel('second');
ylabel('rpm');
subplot(2,3,6);
plot(tnew,q1(:,legIDd,6));  title('theta3 '' (t)');
xlabel('second');
ylabel('rpm');


%2
% legID=2;
% subplot(2,3,1);
% plot(tnew,q2(:,legID,1));
% subplot(2,3,2);
% plot(tnew,q2(:,legID,2));
% subplot(2,3,3);
% plot(tnew,q2(:,legID,3));
% 
% subplot(2,3,4);
% plot(tnew,q2(:,legID,4));
% subplot(2,3,5);
% plot(tnew,q2(:,legID,5));
% subplot(2,3,6);
% plot(tnew,q2(:,legID,6));

