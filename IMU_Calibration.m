%% IMU Calibration

close all
% Import Log File from test. 

test_data1 = csvread('GSDataLog(Test1).csv',3,5);
test_data2 = csvread('GSDataLog2(Test1).csv',3,5);

% Plot Test Data

A_xyz = test_data1(:,1:3);
% [g's] platform orientation in g's on RPY axes

W_xyz = test_data2(:,4:6);
% [deg/s] platform rotation rate in deg/s around XYZ axes

% Plot accel data, input from user to begin data processing
plot(A_xyz)
hold on
legend('1','2','3')
title('Accelerometer Data vs. Test Time')
xlabel('Sample Points')
ylabel('Acceleration (g)')
   
% Plot gyro data, input from user to begin data processing
figure
plot(W_xyz)
hold on
legend('1','2','3')
title('Gyroscope Data vs. Test Time')
xlabel('Sample Points')
ylabel('Angular Rotation (\circ / s)')


%% 3-axis accelerometer
% To calibrate the 3-axis accelerometer, use the 3-axis rotation table to
% conduct a tumble test. The table is rotated on all 6 sides (positive and
% negative of each axis) and rests for 45 seconds each side.

% Description of Test Configurations in Desired Executed Order:
% 1: 0 degrees on all axes (+Z)
% 2: 90 degrees around axis 2 (-X)
% 3: 180 degrees around axis 2 (-Z)
% 4: 270 degrees around axis 2 (+X)
% 5: 90 degrees around axis 1, 270 degrees around axis 2 (+Y)
% 6: 270 degrees around axis 1, 270 degrees around axis 2 (-Y)

disp('ACCEL CALIBRATION POST-PROCESSING')
disp('Input a row vector of the start and end times marking each test leg.')
disp('[SP3 EP3 SN1 EN1 SN3 EN3 SP1 EP1 SP2 EP2 SN2 EN2]')
disp('Select values as close to the ends of each segment as possible')
disp('')

Time_Indices1 = input('Your input from analyzing the graph in Fig. 1: ');

% Begin Calibration calculations.

fapz = mean(A_xyz(Time_Indices1(1:2),3));
fanz = mean(A_xyz(Time_Indices1(5:6),3));

fapx = mean(A_xyz(Time_Indices1(7:8),1));
fanx = mean(A_xyz(Time_Indices1(3:4),1));

fapy = mean(A_xyz(Time_Indices1(9:10),2));
fany = mean(A_xyz(Time_Indices1(11:12),2));

faxpz = mean(A_xyz(Time_Indices1(1:2),1)); 
faxnz = mean(A_xyz(Time_Indices1(5:6),1));

faypz = mean(A_xyz(Time_Indices1(1:2),2)); 
faynz = mean(A_xyz(Time_Indices1(5:6),2));

faxpy = mean(A_xyz(Time_Indices1(9:10),1));
faxny = mean(A_xyz(Time_Indices1(11:12),1));

fazpy = mean(A_xyz(Time_Indices1(9:10),3));
fazny = mean(A_xyz(Time_Indices1(11:12),3));

faypx = mean(A_xyz(Time_Indices1(7:8),2));
faynx = mean(A_xyz(Time_Indices1(3:4),2));

fazpx = mean(A_xyz(Time_Indices1(7:8),3));
faznx = mean(A_xyz(Time_Indices1(3:4),3));

% Begin Bias calculations.

baz = 1/2*(fapz+fanz);
bax = 1/2*(fapx+fanx);
bay = 1/2*(fapy+fany);

ba = [bax;bay;baz];

% Begin Scale Factor calculations.

saz = 1/(2*9.807)*(fapz-fanz)-1;
sax = 1/(2*9.807)*(fapx-fanx)-1;
say = 1/(2*9.807)*(fapy-fany)-1;

% Begin Misalignment calculations  

maxz = 1/(2*9.807)*(faxpz-faxnz);
mayz = 1/(2*9.807)*(faypz-faynz);

maxy = 1/(2*9.807)*(faxpy-faxny);
mazy = 1/(2*9.807)*(fazpy-fazny);

mayx = 1/(2*9.807)*(faypx-faynx);
mazx = 1/(2*9.807)*(fazpx-faznx);

% Accelerometer Calibration Calculation

Ma = [ sax, maxy, maxz;...
      mayx,  say, mayx;...
      mazx, mazy,  saz];
  
disp('Accel Calibration Values')
fprintf('Maxy = %1.6f \n Maxz = %1.6f \n Mayx = %1.6f \n Mayz = %1.6f \n Mazx = %1.6f \n Mazy = %1.6f \n Sax = %1.6f \n Say = %1.6f \n Saz = %1.6f \n',maxy,maxz,mayx,mayz,mazx,mazy,sax,say,saz)
fprintf('AccelBias = \n %1.6f \n %1.6f \n %1.6f \n\n',ba)

% A_xyz_CAL = zeros(size(A_xyz));
% 
% for i = 1:size(test_data1,1);
%     
%     A_xyz_CAL(i,:) = transpose(inv(eye(3)+Ma)*(A_xyz(i,:)'-[bax;bay;baz]));
%     
% end
% 
%% 3-axis Gyroscope Calibration

% To calibrate the 3-axis gyroscope, use the 3-axis rotation table to
% conduct a tumble test. The table is rotated at a constant rate around 
% all 6 axes (positive and negative of each axis) for 45 seconds each side.

% Description of Test Configurations in Desired Executed Order:
% 1: positive, CCW rotation around axis 1 (-Z)
% 2: negative, CW rotation around axis 1 (+Z)
% 3: positive, CCW rotation around axis 2 with axis 1 at 0 degrees (+Y)
% 4: negative, CW rotation around axis 2 with axis 1 at 0 degrees (-Y)
% 5: positive, CCW rotation around axis 2 with axis 1 at 90 degrees (-X)
% 6: negative, CW rotation around axis 2 with axis 1 at 90 degrees (+X)

disp('GYRO CALIBRATION POST-PROCESSING')
disp('Input a row vector of the start and end times marking each test leg.')
disp('[SN3 EN3 SP3 EP3 SP2 EP2 SN2 EN2 SN1 EN1 SP1 EP1]')
disp('Select values as close to the ends of each segment as possible')
disp('')

Time_Indices2 = input('Your input from analyzing the graph in Fig. 2: ');

% Begin Calibration calculations.

fgpz = mean(W_xyz(Time_Indices2(3:4),3));
fgnz = mean(W_xyz(Time_Indices2(1:2),3));

fgpy = mean(W_xyz(Time_Indices2(5:6),2));
fgny = mean(W_xyz(Time_Indices2(7:8),2));

fgpx = mean(W_xyz(Time_Indices2(11:12),1));
fgnx = mean(W_xyz(Time_Indices2(9:10),1));

fgxpz = mean(W_xyz(Time_Indices2(3:4),1));
fgxnz = mean(W_xyz(Time_Indices2(1:2),1)); 

fgypz = mean(W_xyz(Time_Indices2(3:4),2));
fgynz = mean(W_xyz(Time_Indices2(1:2),2)); 

fgxpy = mean(W_xyz(Time_Indices2(5:6),1));
fgxny = mean(W_xyz(Time_Indices2(7:8),1));

fgzpy = mean(W_xyz(Time_Indices2(5:6),3));
fgzny = mean(W_xyz(Time_Indices2(7:8),3));

fgypx = mean(W_xyz(Time_Indices2(11:12),2));
fgynx = mean(W_xyz(Time_Indices2(9:10),2));

fgzpx = mean(W_xyz(Time_Indices2(11:12),3));
fgznx = mean(W_xyz(Time_Indices2(9:10),3));

% Begin Bias calculations.

bgz = 1/2*(fgpz+fgnz);
bgx = 1/2*(fgpx+fgnx);
bgy = 1/2*(fgpy+fgny);

bg = [bgx;bgy;bgz];

% Begin Scale Factor calculations.

sgz = 1/(2*9.807)*(fgpz-fgnz)-1;
sgx = 1/(2*9.807)*(fgpx-fgnx)-1;
sgy = 1/(2*9.807)*(fgpy-fgny)-1;

% Begin Misalignment calculations  

mgxz = 1/(2*9.807)*(fgxpz-fgxnz);
mgyz = 1/(2*9.807)*(fgypz-fgynz);

mgxy = 1/(2*9.807)*(fgxpy-fgxny);
mgzy = 1/(2*9.807)*(fgzpy-fgzny);

mgyx = 1/(2*9.807)*(fgypx-fgynx);
mgzx = 1/(2*9.807)*(fgzpx-fgznx);

% Gyroscope Calibration Calculation

Mg = [ sgx, mgxy, mgxz;...
      mgyx,  sgy, mgyx;...
      mgzx, mgzy,  sgz];

disp('Gyro Calibration Values')
fprintf('Mgxy = %1.6f \n Mgxz = %1.6f \n Mgyx = %1.6f \n Mgyz = %1.6f \n Mgzx = %1.6f \n Mgzy = %1.6f \n Sgx = %1.6f \n Sgy = %1.6f \n Sgz = %1.6f \n',mgxy,mgxz,mgyx,mgyz,mgzx,mgzy,sgx,sgy,sgz)
fprintf('GyroBias = \n %1.6f \n %1.6f \n %1.6f \n\n',bg)

% W_xyz_CAL = zeros(size(W_xyz));
% 
% for i = 1:size(test_data2,1);
%     
%     W_xyz_CAL(i,:) = transpose(inv(eye(3)+Mg)*(W_xyz(i,:)'-[bgx;bgy;bgz]));
%     
% end    

%% Save Data from Calibration

AccelCal.Bias = struct('bAx',bax,'bAy',bay,'bAz',baz);
AccelCal.ScaleFactor = struct('sAx',sax,'sAy',say,'sAz',saz);
AccelCal.Misalignment = struct('mAxy',maxy,'mAxz',maxz,'mAyx',mayx,'mAyz',mayz,'mAzx',mazx,'mAzy',mazy);

GyroCal.Bias = struct('bGx',bgx,'bGy',bgy,'bGz',bgz);
GyroCal.ScaleFactor = struct('sGx',sgx,'sGy',sgy,'sGz',sgz);
GyroCal.Misalignment = struct('mGxy',mgxy,'mGxz',mgxz,'mGyx',mgyx,'mGyz',mgyz,'mGzx',mgzx,'mGzy',mgzy);

save('IMU_Calibration_Data','A_xyz','W_xyz','AccelCal','GyroCal')