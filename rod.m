%%
%Chris Kline
%Detail
%Rodriguez
%%
clc 
%close all 
clear all
dt=0.01;
data =xlsread('GSDataLog.xlsx');
n=1;
%Accelerometer data for first time step (g's)
a=[data(1,2),data(1,3),data(1,4)];
%For purpose of IMU validation on 3-axis rotation table, initialize I as [1;0;0].
i=[1;0;0]';
%The accelerometer reading forms the initial K vector, pointing down.
k=a; 
%Compute the initial J vector, which is the cross product of the K and I
j=cross(k,i);
%Recompute the I vector from the cross product of the J and K vectors.This
%makes it fully orthogonal
i=cross(j,k);
%Normalize the i j and k vectors
i=i/norm(i);
j=j/norm(j);
k=k/norm(k);
%Initialize the DCM matrix from the I, J, and K vectors.
DCM=[i;j;k];
%Pull gyro readings, outputed in deg/s, then converted to rad/sec
w=[data(2:end,5:7)]'.*(pi/180);
%Initialize Yaw
yaw=zeros(1,length(data)); 
while n<=length(w) 
    %Calculate normalized gyro vector
    wnorm=w(:,n)/norm(w(:,n));
    %Calculate Skew Symetric Matrix
    K=[0             -wnorm(3)       wnorm(2);...
      wnorm(3)       0               -wnorm(1);...
      -wnorm(2)      wnorm(1)           0];
    %Calculate Theta from multiplying angular velocity by dt
    theta = dt*norm(w(:,n)); 
    %calculate 3X3 Rotation matrix
    C=eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2; 
    %Update DCM
    DCM=DCM*C; 
    %Calculate Yaw from the DCM
    yaw(n)=atan2(DCM(2,1),DCM(1,1)); 
    %Calculate Roll from the DCM
    roll(n)=atan2(DCM(3,2),DCM(3,3));
    %Calculate Pitch from the DCM
    pitch(n)=-asin(DCM(3,1));
    %Convert Roll Pitch and Yaw to degrees
    roll(n)=roll(n)*(180/pi); 
    pitch(n)=pitch(n)*(180/pi); 
    yaw(n)=yaw(n)*(180/pi); 
    n=n+1;
end
%Plot Yaw
plot(roll) 
hold on
plot(pitch)
plot(yaw)
%Plot the results from the C code
plot(data(:,11:13))