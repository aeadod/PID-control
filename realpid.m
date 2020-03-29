clc
clear
close
ts = 0.001;%时间步长
sys = tf(523500,[1,87.35,10470,0]);%传递函数
dsys = c2d(sys,ts,'z');%离散化
[num,den] = tfdata(dsys,'v');%取分子分母
[y0,t,~] = step(sys,0.5);%Step函数求解系统的阶跃响应
% figure(1);
% plot([0 0.5],[1 1],'b',t,y0,'r');%阶跃响应曲线

u_1 = 0.0;
u_2 = 0.0;
u_3 = 0.0;
y_1 = 0;
y_2 = 0;
y_3 = 0;
x = [0 0 0]';
x2_1 = 0;

kp = 0.6;%比例环节的系数
ki = 0.03;%积分环节的系数
kd = 0.01;%微分环节的系数

error_1 = 0;
for k = 1:1:500
time(k) = k*ts;
r(k)= 1.0;                             
%跟踪阶跃信号
u(k)= kp*x(1)+kd*x(2)+ki*x(3);          %PID控制器

%专家控制规则
if abs(x(1))>0.8                    			 %Rule1:误差值特别大时，控制器按最大输出
	u(k) = 0.45;                                    
elseif abs(x(1))>0.40
	u(k) = 0.40;
elseif abs(x(1))>0.20 
	u(k) = 0.12;    
elseif abs(x(1))>0.01
	u(k) = 0.10;
end
if x(1)*x(2)>0||(x(2)==0)             			%Rule2:当e(k)?e(k)>0或者?e(k)=0时
	if abs(x(1))>=0.05                          %偏差较大，实行较强的控制作用
		u(k)=u_1+2*kp*x(1); 
	else u(k)=u_1+0.4*kp*x(1);                  %偏差绝对值本身并不是很大，考虑控制器实施一般的控制作用
    end                                         %只需要扭转偏差的变化趋势，使其向偏差绝对值减小的方向变化即可。
end
if (x(1)*x(2)<0&&x(2)*x2_1>0)||(x(1)==0)	 	%Rule3：偏差的绝对值向减小的方向变化，或者已经达到平衡状态，                                                        
	u(k)=u(k);                                  %此时保持控制器输出不变即可。
end
if x(1)*x(2)<0&&x(2)*x2_1<0              		%Rule4:偏差处于极值状态
	if abs(x(1))>=0.05                          %如果此时偏差的绝对值较大，实施较强控制作用，这里k1取2。
		u(k)=u_1+2*kp*error_1;                  
	else     
		u(k)=u_1+0.6*kp*error_1;                %如果此时偏差的绝对值较小，实施较弱控制作用，这里k2取0.6。
	end
end
if abs(x(1))<=0.001                             %Rule5:偏差绝对值很小，此时要引入积分作用，实施PI控制。
	u(k)=0.5*x(1)+0.010*x(3); 
end
 
%限制控制器的输出
if u(k)>=10     
	u(k) = 10;
end
if u(k) <= -10
	u(k) = -10;
end   

%线性模型
y(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
error(k) = r(k)-y(k);     
                 
%返回参数
u_3 =u_2;
u_2 =u_1;
u_1 =u(k);
y_3 =y_2;
y_2 =y_1;
y_1 =y(k);  
x(1)= error(k);                % P
x(2)= (error(k)-error_1)/ts;   % D
x(3)= x(3)+error(k)*ts;        % I
x2_1= x(2);
error_1 = error(k);
end
figure(1);  
plot(time,r,'b',time,y,'r');   
xlabel('time(s)');ylabel('r,y');   

