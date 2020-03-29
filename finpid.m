clc
clear
close
PID=[0.6,0.03,0.01;
    0.6,0.13,0.01; 
    0.6,0.33,0.01;
    0.6,1.03,0.01;
    ];

ts = 0.001;%ʱ�䲽��
sys = tf(523500,[1,87.35,10470,0]);%���ݺ���
dsys = c2d(sys,ts,'z');%��ɢ��
[num,den] = tfdata(dsys,'v');%ȡ���ӷ�ĸ
[y0,t,~] = step(sys,0.5);%Step�������ϵͳ�Ľ�Ծ��Ӧ

for pid=1:1:4

u_1 = 0.0;
u_2 = 0.0;
u_3 = 0.0;
y_1 = 0;
y_2 = 0;
y_3 = 0;
x = [0 0 0]';
kp=PID(pid,1);    
ki=PID(pid,2);
kd=PID(pid,3);
error_1 = 0;

for k = 1:1:500
time(k) = k*ts;
r(k)= 1.0;                              %��Ծ����
u(k)= kp*x(1)+kd*x(2)+ki*x(3);          %PID������

if u(k)>=10                             %ʵ�ֳ�������
	u(k) = 10;
end
if u(k) <= -10
	u(k) = -10;
end 

%ʵ����������
y(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
error(k) = r(k)-y(k);     
                 
%���ز���
u_3 =u_2;
u_2 =u_1;
u_1 =u(k);
y_3 =y_2;
y_2 =y_1;
y_1 =y(k);  
x(1)= error(k);                % P
x(2)= (error(k)-error_1)/ts;   % D
x(3)= x(3)+error(k)*ts;        % I
error_1 = error(k);
end

subplot(2,2,pid);
p1=plot(time,r,'-.');xlim([0,0.5]);ylim([0,2]);hold on;
p2=plot(time,y,'--');xlim([0,0.5]);ylim([0,2])
title(['Kp=',num2str(kp),' Ki=',num2str(ki),' Kd= ',num2str(kd)]);
hold on;
end
