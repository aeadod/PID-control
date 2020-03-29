clear
close 
ts=0.001;                 %����ʱ��
sys = tf(523500,[1,87.35,10470,0]);%���ݺ���
dsys = c2d(sys,ts,'z');%��ɢ��
[num,den] = tfdata(dsys,'v');%ȡ���ӷ�ĸ

u_1=0.0;
u_2=0.0;
u_3=0.0;
y_3=0.0;
y_1=0.0;
y_2=0.0;

x=[0,0,0]';
x2_1=0;
kp=1.22;ki=0.13;kd=0;  %��ʼ��PID  
error_1 = 0;

for k=1:1:1000
time(k)=k*ts;                        %��������
r(k)=1;                             %Step Signal 
du(k)=kp*x(1)+kd*x(2)+ki*x(3);      %PID Controller   ����ϵ��  
u(k)=u_1+du(k);                     %Restricting the output of controller
if u(k)>=10      
   u(k)=10;
end
if u(k)<=-10
   u(k)=-10;
end
%Linear model
y(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
error(k) = r(k)-y(k);

u_3 =u_2;
u_2 =u_1;
u_1 =u(k);
y_3 =y_2;
y_2 =y_1;
y_1 =y(k);  

x(1)= error(k);                % P
x2_1= x(2);
x(2)= (error(k)-error_1)/ts;   % D
x(3)= x(3)+error(k)*ts;        % I
    
error_1 = error(k);         
end
figure(1);
plot(time,r,'b',time,y,'r');                        %���� ��ʵ�ʿ������
xlabel('time(s)'),ylabel('r,y'); 
% figure(2);
% plot(time,error,'r')                                     %�������������������
% xlabel('time(s)');ylabel('error')


