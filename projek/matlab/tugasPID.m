clear all

%plant kecepatan motor dc
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
s = tf('s');
motor = K/((J*s+b)*(L*s+R)+K^2);

%PID
kp=10;
ki=20;
kd=1;
pid= kp + ki/s +kd*s;

%A/D motor dc
Ts = 0.05;
d_motor = c2d(motor,Ts,'zoh');
zpk(d_motor);
dpid= c2d (pid,Ts,'tustin');
zpk(dpid)

%system closeloop
sys = feedback(d_motor,1)
z = tf('z',Ts);
d_pid_com = dpid/(z+0.82);
sys_cl=feedback(d_pid_com*d_motor,1)


yk = 0;
yk_1 = 0;
sum_ek=0;
ek_1=0;
uk = 1;
uk_1= 0;
uk_2 = 0;
uk_3 = 0;

y=[];

k=0;
Ts=1;
runtime=100;
n_iteration=runtime/Ts;
sp=1;

% while k<n_iteration
%     yk = 0.904*yk_1+0.95*uk_3; %plant
%     ek=sp-yk;
%     sum_ek=sum_ek+ek;
%     div_ek=(ek-ek_1)/Ts;
%     uk=kp*ek+ki*sum_ek+kd*div_ek;
%     ek_1=ek;
%     yk_1=yk;
%     uk_3=uk_2;
%     uk_2=uk_1;
%     uk_1=uk;
%     
%     y=[y;yk];
%     k=k+1;
% end
% stem(y);