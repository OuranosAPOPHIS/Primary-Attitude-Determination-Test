%% IMU Validation

% Import Log File from Validation Test.

test_data = csvread('GSDataLogVTest4_4.csv',3,14);

%% Post-Processing of Roll and Pitch Measurements

roll = test_data(:,1);
% measured roll during validation test

pitch = test_data(:,2);
% measured pitch during validation test

%% Vectors of Angles Tested

x = linspace(0,length(roll),1001);
yp5p = 5.*ones(size(x))+0.5;
yp5n = 5.*ones(size(x))-0.5;
yp15p = 15.*ones(size(x))+0.5;
yp15n = 15.*ones(size(x))-0.5;
yp30p = 30.*ones(size(x))+0.5;
yp30n = 30.*ones(size(x))-0.5;
yn5p = -5.*ones(size(x))+0.5;
yn5n = -5.*ones(size(x))-0.5;
yn15p = -15.*ones(size(x))+0.5;
yn15n = -15.*ones(size(x))-0.5;
yn30p = -30.*ones(size(x))+0.5;
yn30n = -30.*ones(size(x))-0.5;

figure
hold on
title('Roll')
xlabel('Sample')
ylabel('Roll (\circ)')
plot(roll,'.k')
plot(x,yp5p,x,yp15p,x,yp30p,x,yn30p,x,yn15p,x,yn5p)
plot(x,yp5n,x,yp15n,x,yp30n,x,yn30n,x,yn15n,x,yn5n)

figure
hold on
title('Pitch')
xlabel('Sample')
ylabel('Pitch (\circ)')
plot(pitch,'.k')
plot(x,yp5p,x,yp15p,x,yp30p,x,yn30p,x,yn15p,x,yn5p)
plot(x,yp5n,x,yp15n,x,yp30n,x,yn30n,x,yn15n,x,yn5n)
