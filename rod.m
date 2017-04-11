%%
%Chris Kline
%Detail
%Rodriguez
%%
clc 
close all 
clear all
dt=0.01;
data =xlsread('GSDataLog.xlsx');
n=1;
a=[data(1,2),data(1,3),data(1,4)];
i=[1;0;0]';
k=a;
j=cross(k,i);
i=cross(j,k);
i=i/norm(i);
j=j/norm(j);
k=k/norm(k);
DCM=[i;j;k];
w=[data(2:end,5:7)]'.*(pi/180);
yaw=zeros(1,length(data));
while n<=length(w) 
wnorm=w(:,n)/norm(w(:,n));
K=[0 -wnorm(3) wnorm(2);wnorm(3) 0 -wnorm(1);-wnorm(2) wnorm(1) 0];
C=eye(3)+sin(dt*norm(w(:,n)))*K+(1-cos(dt*norm(w(:,n))))*(K*K);
DCM=DCM*C;
yaw(n)=atan2(DCM(2,1),DCM(1,1));
yaw(n)=yaw(n)*(180/pi);
n=n+1;
end
plot(yaw)