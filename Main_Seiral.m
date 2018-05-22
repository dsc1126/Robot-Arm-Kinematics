%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Robotic Fundamentals Coursework    %%%%
%%%% Programmed by Dong Shichao(12034357) %%%%
%%%%           December 2016              %%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

global rad2deg deg2rad inch2mm L1 L2 L3 L4 L5 theta1 theta2 theta3 theta4 theta5
global theta1_IK theta2a_IK theta3a_IK theta4a_IK theta2b_IK theta3b_IK theta4b_IK xTarget yTarget zTarget Phi
global timeslot EnableVelocityPrint
%global z6 x6 z4 x4 s alpha cos_beta sin_beta beta total_stage EE_kinematics J 
deg2rad = pi/180; %convert degrees to rads
rad2deg = 180/pi; %convert rads to degrees
inch2mm = 25.4;

%% Program options
PlotLinks = 1;
PlotWorkspace = 0;
InverseKinematics = 0;
executePath = 5;
EnableVelocityPrint = 0; % 0 is off; 1 is to print EE velocity; 2 is to print EE velocity and joint anguler velocity;

if (PlotWorkspace==1||executePath~=0)
    PlotLinks = 0;
end

%% variables such as link length and thetas
%link lengths
L1 = 2.7*inch2mm; %L0+L1
L2 = 4.75*inch2mm;
L3 = 5*inch2mm;
L4 = 1.5*inch2mm;
L5 = 1.8*inch2mm;

% thetas
theta1 = -11.31*deg2rad; % Limit from -90 to 90 
theta2 = 32.94*deg2rad; % Limit from 0 to 180
theta3 = -74.91*deg2rad; % Limit from -165 to 0
theta4 = -3.04*deg2rad; % Limit from -90 to 90
theta5 = 0*deg2rad; % Limit from -90 to 90

%% DH table
DH1 = [0,(pi/2),L1,theta1];
DH2 = [L2,0,0,theta2];
DH3 = [L3,0,0,theta3];
DH4 = [0,-(pi/2),0,theta4-(pi/2)];
DH5 = [0,0,L4+L5,theta5];

%% transformation matrix ' s
BL = get_dh_matrix_distal([0,0,0,0]); %Base Link
T01 = get_dh_matrix_distal(DH1);
T12 = get_dh_matrix_distal(DH2);
T23 = get_dh_matrix_distal(DH3);
T34 = get_dh_matrix_distal(DH4);
T45 = get_dh_matrix_distal(DH5);

% calculate the total transformation matrix
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;

%%
hold on
grid on;
title ('AL5B Robot Arm Simulation');
ylabel('Y(mm)'),xlabel('X(mm)'),zlabel('Z(mm)')
daspect([1 1 1]) %keep aspect ratio the same
view([0,-1,0])
axis([-500 500 -500 500 -500 500]);

if PlotLinks == 1
OOx = [0;T01(1,4);T02(1,4);T03(1,4);T04(1,4);T05(1,4)];
OOy = [0;T01(2,4);T02(2,4);T03(2,4);T04(2,4);T05(2,4)];
OOz = [0;T01(3,4);T02(3,4);T03(3,4);T04(3,4);T05(3,4)];
plot3(OOx,OOy,OOz,'b','LineWidth',2);

text(BL(1,4),BL(2,4),BL(3,4),' 0 ');
text(T01(1,4),T01(2,4),T01(3,4),' 1 ');
text(T02(1,4),T02(2,4),T02(3,4),' 2 ');
text(T03(1,4),T03(2,4),T03(3,4),' 3 ');
text(T04(1,4),T04(2,4),T04(3,4),'   4 ');
text(T05(1,4),T05(2,4),T05(3,4),' 5 ');

scatter3(0,0,0,15,[0,1,1],'filled');
scatter3(T01(1,4),T01(2,4),T01(3,4),15,[0,1,1],'filled');
scatter3(T02(1,4),T02(2,4),T02(3,4),15,[0,1,1],'filled');
scatter3(T03(1,4),T03(2,4),T03(3,4),15,[0,1,1],'filled');
scatter3(T04(1,4),T04(2,4),T04(3,4),15,[0,1,1],'filled');
scatter3(T05(1,4),T05(2,4),T05(3,4),20,[1,0,0],'filled');
end

if PlotWorkspace == 1
X = [ ] ;
Y = [ ] ;
Z = [ ] ;
C = [ ] , [ ] ;
for i =-90:2:90,
fprintf ( ' at iteration : %d \n ' , i ) %progress report
for j =0:10:180,
for k=-165:10:0,
for l =-90:5:90,
theta1=i*deg2rad;
theta2=j*deg2rad;
theta3=k*deg2rad;
theta4=l*deg2rad ;

DH1 = [0,(pi/2),L1,theta1];
DH2 = [L2,0,0,theta2];
DH3 = [L3,0,0,theta3];
DH4 = [0,-(pi/2),0,theta4-(pi/2)];
DH5 = [0,0,L4+L5,theta5];

T01 = get_dh_matrix_distal(DH1);
T02 = T01*get_dh_matrix_distal(DH2);
T03 = T02*get_dh_matrix_distal(DH3);
T04 = T03*get_dh_matrix_distal(DH4);
T05 = T04*get_dh_matrix_distal(DH5);

C(end+1,1) = 1-(j/180); %set red value from joint 2
C(end,2) = ((k+165)/165);%set green value from joint 3
C(end,3) = (l+90/180); %set blue value from joint 4
X(end+1) = T05(1,4);
Y(end+1) = T05(2,4);
Z(end+1) = T05(3,4);

end
end
end
end
scatter3(X,Y,Z,20,C,'filled');
hold on
end

if InverseKinematics == 1

xTarget = 250;
yTarget = -50;
zTarget = -10;
Phi = -45; %manualy set the angles of the gripper relative to ground

[theta1_IK,theta2a_IK,theta3a_IK,theta4a_IK,theta2b_IK,theta3b_IK,theta4b_IK] = solve_IK(xTarget,yTarget,zTarget,Phi);
if theta1_IK>=-90&&theta1_IK<=90&&theta2a_IK>=0&&theta2a_IK<=180&&theta3a_IK>=-165&&theta3a_IK<=0&&theta4a_IK>=-90&&theta4a_IK<=90%All angles must within available range
fprintf('Input angles:\ttheta1=%.2f\ttheta2=%.2f\ttheta3=%.2f\ttheta4=%.2f\n',theta1*rad2deg,theta2*rad2deg,theta3*rad2deg,theta4*rad2deg)
fprintf('End efector position:\tx=%.2f\ty=%.2f\tz=%.2f\n',T05(1,4),T05(2,4),T05(3,4))
fprintf('--------------------------------------------------------------------------------------------\n')
fprintf('Target position:\txTarget=%.2f\tyTarget=%.2f\tzTarget=%.2f\tPhi=%.2f\n',xTarget,yTarget,zTarget,Phi)
fprintf('Solution 1:\ttheta1=%.2f\ttheta2=%.2f\ttheta3=%.2f\ttheta4=%.2f\n',theta1_IK,theta2a_IK,theta3a_IK,theta4a_IK)
fprintf('Solution 2:\ttheta1=%.2f\ttheta2=%.2f\ttheta3=%.2f\ttheta4=%.2f\n',theta1_IK,theta2b_IK,theta3b_IK,theta4b_IK)
else
fprintf('There is no solution for target position!!!')
end
end


if executePath == 1  %

view([1,1,1])
axis([-500 500 -500 500 -500 500]);

timeslot=0.1;

%--------------- (Stage 1) -----------------  
for i =-90:5:90,
j = 45;
k = -60;
l = 30;

theta1=i*deg2rad;
theta2=j*deg2rad;
theta3=k*deg2rad;
theta4=l*deg2rad ;

DH1 = [0,(pi/2),L1,theta1];
DH2 = [L2,0,0,theta2];
DH3 = [L3,0,0,theta3];
DH4 = [0,-(pi/2),0,theta4-(pi/2)];
DH5 = [0,0,L4+L5,theta5];

T01 = get_dh_matrix_distal(DH1);
T02 = T01*get_dh_matrix_distal(DH2);
T03 = T02*get_dh_matrix_distal(DH3);
T04 = T03*get_dh_matrix_distal(DH4);
T05 = T04*get_dh_matrix_distal(DH5);

OOx = [0;T01(1,4);T02(1,4);T03(1,4);T04(1,4);T05(1,4)];
OOy = [0;T01(2,4);T02(2,4);T03(2,4);T04(2,4);T05(2,4)];
OOz = [0;T01(3,4);T02(3,4);T03(3,4);T04(3,4);T05(3,4)];
lines = plot3(OOx,OOy,OOz,'b','LineWidth',2);

text1 = text(BL(1,4),BL(2,4),BL(3,4),' 0 ');
text2 = text(T01(1,4),T01(2,4),T01(3,4),' 1 ');
text3 = text(T02(1,4),T02(2,4),T02(3,4),' 2 ');
text4 = text(T03(1,4),T03(2,4),T03(3,4),' 3 ');
text5 = text(T04(1,4),T04(2,4),T04(3,4),'   4 ');
text6 = text(T05(1,4),T05(2,4),T05(3,4),' 5 ');

dot1 = scatter3(0,0,0,15,[0,1,1],'filled');
dot2 = scatter3(T01(1,4),T01(2,4),T01(3,4),15,[0,1,1],'filled');
dot3 = scatter3(T02(1,4),T02(2,4),T02(3,4),15,[0,1,1],'filled');
dot4 = scatter3(T03(1,4),T03(2,4),T03(3,4),15,[0,1,1],'filled');
dot5 = scatter3(T04(1,4),T04(2,4),T04(3,4),15,[0,1,1],'filled');
dot6 = scatter3(T05(1,4),T05(2,4),T05(3,4),20,[1,0,0],'filled');
drawnow

fprintf('x= %.2f \t y= %.2f \t z= %.2f \n',T05(1,4),T05(2,4),T05(3,4));

delay(timeslot);

delete(lines);
delete(text1);
delete(text2);
delete(text3);
delete(text4);
delete(text5);
delete(text6);
delete(dot2);
delete(dot3);
delete(dot4);
delete(dot5);
end

%--------------- (Stage 2) -----------------  
for j =45:5:165,
i = 90;
k = -60;
l = 30;

theta1=i*deg2rad;
theta2=j*deg2rad;
theta3=k*deg2rad;
theta4=l*deg2rad ;

DH1 = [0,(pi/2),L1,theta1];
DH2 = [L2,0,0,theta2];
DH3 = [L3,0,0,theta3];
DH4 = [0,-(pi/2),0,theta4-(pi/2)];
DH5 = [0,0,L4+L5,theta5];

T01 = get_dh_matrix_distal(DH1);
T02 = T01*get_dh_matrix_distal(DH2);
T03 = T02*get_dh_matrix_distal(DH3);
T04 = T03*get_dh_matrix_distal(DH4);
T05 = T04*get_dh_matrix_distal(DH5);

OOx = [0;T01(1,4);T02(1,4);T03(1,4);T04(1,4);T05(1,4)];
OOy = [0;T01(2,4);T02(2,4);T03(2,4);T04(2,4);T05(2,4)];
OOz = [0;T01(3,4);T02(3,4);T03(3,4);T04(3,4);T05(3,4)];
lines = plot3(OOx,OOy,OOz,'b','LineWidth',2);

text1 = text(BL(1,4),BL(2,4),BL(3,4),' 0 ');
text2 = text(T01(1,4),T01(2,4),T01(3,4),' 1 ');
text3 = text(T02(1,4),T02(2,4),T02(3,4),' 2 ');
text4 = text(T03(1,4),T03(2,4),T03(3,4),' 3 ');
text5 = text(T04(1,4),T04(2,4),T04(3,4),'   4 ');
text6 = text(T05(1,4),T05(2,4),T05(3,4),' 5 ');

dot1 = scatter3(0,0,0,15,[0,1,1],'filled');
dot2 = scatter3(T01(1,4),T01(2,4),T01(3,4),15,[0,1,1],'filled');
dot3 = scatter3(T02(1,4),T02(2,4),T02(3,4),15,[0,1,1],'filled');
dot4 = scatter3(T03(1,4),T03(2,4),T03(3,4),15,[0,1,1],'filled');
dot5 = scatter3(T04(1,4),T04(2,4),T04(3,4),15,[0,1,1],'filled');
dot6 = scatter3(T05(1,4),T05(2,4),T05(3,4),20,[1,0,0],'filled');
drawnow

fprintf('x= %.2f \t y= %.2f \t z= %.2f \n',T05(1,4),T05(2,4),T05(3,4));

delay(timeslot);

delete(lines);
delete(text1);
delete(text2);
delete(text3);
delete(text4);
delete(text5);
delete(text6);
delete(dot2);
delete(dot3);
delete(dot4);
delete(dot5);
end

%--------------- (Stage 3) -----------------  
for k =60:5:150,
k = -k;    
i = 90;
j = 165;
l = 30;

theta1=i*deg2rad;
theta2=j*deg2rad;
theta3=k*deg2rad;
theta4=l*deg2rad ;

DH1 = [0,(pi/2),L1,theta1];
DH2 = [L2,0,0,theta2];
DH3 = [L3,0,0,theta3];
DH4 = [0,-(pi/2),0,theta4-(pi/2)];
DH5 = [0,0,L4+L5,theta5];

T01 = get_dh_matrix_distal(DH1);
T02 = T01*get_dh_matrix_distal(DH2);
T03 = T02*get_dh_matrix_distal(DH3);
T04 = T03*get_dh_matrix_distal(DH4);
T05 = T04*get_dh_matrix_distal(DH5);

OOx = [0;T01(1,4);T02(1,4);T03(1,4);T04(1,4);T05(1,4)];
OOy = [0;T01(2,4);T02(2,4);T03(2,4);T04(2,4);T05(2,4)];
OOz = [0;T01(3,4);T02(3,4);T03(3,4);T04(3,4);T05(3,4)];
lines = plot3(OOx,OOy,OOz,'b','LineWidth',2);

text1 = text(BL(1,4),BL(2,4),BL(3,4),' 0 ');
text2 = text(T01(1,4),T01(2,4),T01(3,4),' 1 ');
text3 = text(T02(1,4),T02(2,4),T02(3,4),' 2 ');
text4 = text(T03(1,4),T03(2,4),T03(3,4),' 3 ');
text5 = text(T04(1,4),T04(2,4),T04(3,4),'   4 ');
text6 = text(T05(1,4),T05(2,4),T05(3,4),' 5 ');

dot1 = scatter3(0,0,0,15,[0,1,1],'filled');
dot2 = scatter3(T01(1,4),T01(2,4),T01(3,4),15,[0,1,1],'filled');
dot3 = scatter3(T02(1,4),T02(2,4),T02(3,4),15,[0,1,1],'filled');
dot4 = scatter3(T03(1,4),T03(2,4),T03(3,4),15,[0,1,1],'filled');
dot5 = scatter3(T04(1,4),T04(2,4),T04(3,4),15,[0,1,1],'filled');
dot6 = scatter3(T05(1,4),T05(2,4),T05(3,4),20,[1,0,0],'filled');
drawnow

fprintf('x= %.2f \t y= %.2f \t z= %.2f \n',T05(1,4),T05(2,4),T05(3,4));

delay(timeslot);

delete(lines);
delete(text1);
delete(text2);
delete(text3);
delete(text4);
delete(text5);
delete(text6);
delete(dot2);
delete(dot3);
delete(dot4);
delete(dot5);
end

%Plot last position
OOx = [0;T01(1,4);T02(1,4);T03(1,4);T04(1,4);T05(1,4)];
OOy = [0;T01(2,4);T02(2,4);T03(2,4);T04(2,4);T05(2,4)];
OOz = [0;T01(3,4);T02(3,4);T03(3,4);T04(3,4);T05(3,4)];
lines = plot3(OOx,OOy,OOz,'b','LineWidth',2);

text1 = text(BL(1,4),BL(2,4),BL(3,4),' 0 ');
text2 = text(T01(1,4),T01(2,4),T01(3,4),' 1 ');
text3 = text(T02(1,4),T02(2,4),T02(3,4),' 2 ');
text4 = text(T03(1,4),T03(2,4),T03(3,4),' 3 ');
text5 = text(T04(1,4),T04(2,4),T04(3,4),'   4 ');
text6 = text(T05(1,4),T05(2,4),T05(3,4),' 5 ');

dot1 = scatter3(0,0,0,15,[0,1,1],'filled');
dot2 = scatter3(T01(1,4),T01(2,4),T01(3,4),15,[0,1,1],'filled');
dot3 = scatter3(T02(1,4),T02(2,4),T02(3,4),15,[0,1,1],'filled');
dot4 = scatter3(T03(1,4),T03(2,4),T03(3,4),15,[0,1,1],'filled');
dot5 = scatter3(T04(1,4),T04(2,4),T04(3,4),15,[0,1,1],'filled');
dot6 = scatter3(T05(1,4),T05(2,4),T05(3,4),20,[1,0,0],'filled');

end

if executePath == 2 %efficient path

view([1,1,1])
axis([-200 200 -200 200 0 400]);

timeslot=0.1;
x=150;
y=0;%100
z=200;%300
pitch =90;
time=2;
    
% Just to print the initial status
theta1 = -45*deg2rad; % Limit from -90 to 90 
theta2 = 169.16*deg2rad; % Limit from 0 to 180
theta3 = -68.76*deg2rad; % Limit from -165 to 0
theta4 = -10.4*deg2rad; % Limit from -90 to 90
theta5 = 0*deg2rad; % Limit from -90 to 90
x_start = -100;
y_start = 100;
z_start = 300;
fprintf('x= %.2f \t y= %.2f \t z= %.2f \t',x_start,y_start,z_start);
fprintf('1= %.2f \t 2= %.2f \t 3= %.2f \t 4= %.2f \n',theta1*rad2deg,theta2*rad2deg,theta3*rad2deg,theta4*rad2deg);

UpdateArm1(x,y,z,pitch,time,1); 
end


if executePath == 3 %planned trajectory (Square)
view([1,1,1])
axis([-100 200 -100 200 0 350]); 
stage_no=11;

x=linspace(100,150,stage_no);   
y=linspace(100,100,stage_no);
z=linspace(300,300,stage_no);
pitch =45;
time=1;
UpdateArm2(x,y,z,pitch,time,0); 

x=linspace(150,150,stage_no);    
y=linspace(100,50,stage_no);
z=linspace(300,300,stage_no);
pitch =45;
time=1;
UpdateArm2(x,y,z,pitch,time,0);

x=linspace(150,100,stage_no);   
y=linspace(50,50,stage_no);
z=linspace(300,300,stage_no);
pitch =45;
time=1;
UpdateArm2(x,y,z,pitch,time,0); 

x=linspace(100,100,stage_no);   
y=linspace(50,100,stage_no);
z=linspace(300,300,stage_no);
pitch =45;
time=1;
UpdateArm2(x,y,z,pitch,time,1);
end

if executePath == 4 %Draw a STAR on paper

view([1,1,1])
axis([-100 200 -100 200 0 350]); 
stage_no=21;

plane1=[-50,-50,310];
plane2=[-50,150,310];
plane3=[150,150,310];
plane4=[150,-50,310];

plane_x = [plane1(1) plane2(1) plane3(1) plane4(1)];
plane_y = [plane1(2) plane2(2) plane3(2) plane4(2)];
plane_z = [plane1(3) plane2(3) plane3(3) plane4(3)];
plane = fill3(plane_x,plane_y,plane_z,1);
set(plane,'facealpha',0.5);

x=linspace(50,79.39,stage_no);   
y=linspace(100,9.55,stage_no);
z=linspace(310,310,stage_no);
pitch=45;
time=1;
UpdateArm2(x,y,z,pitch,time,0); 

x=linspace(79.39,2.45,stage_no);    
y=linspace(9.55,65.45,stage_no);
z=linspace(310,310,stage_no);
pitch=45;
time=1;
UpdateArm2(x,y,z,pitch,time,0);

x=linspace(2.45,97.55,stage_no);   
y=linspace(65.45,65.45,stage_no);
z=linspace(310,310,stage_no);
pitch=45;
time=1;
UpdateArm2(x,y,z,pitch,time,0); 

x=linspace(97.55,20.61,stage_no);   
y=linspace(65.45,9.55,stage_no);
z=linspace(310,310,stage_no);
pitch=45;
time=1;
UpdateArm2(x,y,z,pitch,time,0);

x=linspace(20.61,50,stage_no);   
y=linspace(9.55,100,stage_no);
z=linspace(310,310,stage_no);
pitch=45;
time=1;
UpdateArm2(x,y,z,pitch,time,1);
end
    
if executePath == 5 %planned trajectory (Obstacle Avoidance)

obstacle1=[-100,100,0];
obstacle2=[0,0,0];
obstacle3=[100,100,0];
obstacle4=[0,200,0];
obstacle5=[-100,100,400];
obstacle6=[0,0,400];
obstacle7=[100,100,400];
obstacle8=[0,200,400];

obstacle_x = [obstacle1(1) obstacle2(1) obstacle3(1) obstacle4(1)];
obstacle_y = [obstacle1(2) obstacle2(2) obstacle3(2) obstacle4(2)];
obstacle_z = [obstacle1(3) obstacle2(3) obstacle3(3) obstacle4(3)];
fill3(obstacle_x,obstacle_y,obstacle_z,1);
obstacle_x = [obstacle5(1) obstacle6(1) obstacle7(1) obstacle8(1)];
obstacle_y = [obstacle5(2) obstacle6(2) obstacle7(2) obstacle8(2)];
obstacle_z = [obstacle5(3) obstacle6(3) obstacle7(3) obstacle8(3)];
fill3(obstacle_x,obstacle_y,obstacle_z,1);
obstacle_x = [obstacle1(1) obstacle2(1) obstacle6(1) obstacle5(1)];
obstacle_y = [obstacle1(2) obstacle2(2) obstacle6(2) obstacle5(2)];
obstacle_z = [obstacle1(3) obstacle2(3) obstacle6(3) obstacle5(3)];
fill3(obstacle_x,obstacle_y,obstacle_z,1);
obstacle_x = [obstacle2(1) obstacle3(1) obstacle7(1) obstacle6(1)];
obstacle_y = [obstacle2(2) obstacle3(2) obstacle7(2) obstacle6(2)];
obstacle_z = [obstacle2(3) obstacle3(3) obstacle7(3) obstacle6(3)];
fill3(obstacle_x,obstacle_y,obstacle_z,1);
obstacle_x = [obstacle3(1) obstacle4(1) obstacle8(1) obstacle7(1)];
obstacle_y = [obstacle3(2) obstacle4(2) obstacle8(2) obstacle7(2)];
obstacle_z = [obstacle3(3) obstacle4(3) obstacle8(3) obstacle7(3)];
fill3(obstacle_x,obstacle_y,obstacle_z,1);
obstacle_x = [obstacle4(1) obstacle1(1) obstacle1(1) obstacle8(1)];
obstacle_y = [obstacle4(2) obstacle1(2) obstacle1(2) obstacle8(2)];
obstacle_z = [obstacle4(3) obstacle1(3) obstacle1(3) obstacle8(3)];
fill3(obstacle_x,obstacle_y,obstacle_z,1);

%--------------- (Stage 1) -----------------    
view([1,-1.5,2])
axis([-200 200 -200 200 0 400]); 
stage_no=11;

timeslot=0.1;
x=linspace(-150,-100,stage_no);   
y=linspace(100,100,stage_no);
z=linspace(300,300,stage_no);
pitch=90;
time=1;
UpdateArm2(x,y,z,pitch,time,0); 

%--------------- (Stage 2) -----------------

x=100;
y=100;
z=300;
pitch=90;
time=10;
UpdateArm1(x,y,z,pitch,time,0); 

%--------------- (Stage 3) -----------------
x=linspace(100,150,stage_no);   
y=linspace(100,100,stage_no);
z=linspace(300,300,stage_no);
pitch=90;
time=1;
UpdateArm2(x,y,z,pitch,time,1); 
end

