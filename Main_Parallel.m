%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Robotic Fundamentals Coursework    %%%%
%%%% Programmed by Dong Shichao(12034357) %%%%
%%%%           December 2016              %%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

global rad2deg deg2rad inch2mm SA L r_plt r_base Cx Cy Alpha 
global Bx By PB1x PB1y PB2x PB2y PB3x PB3y PP1x PP1y PP2x PP2y PP3x PP3y theta1 theta2 theta3 d1 d2 d3 dmax dmin
global beta1 beta2 beta3 gamma1a gamma1b gamma2a gamma2b gamma3a gamma3b 
global M1ax M1ay M2ax M2ay M3ax M3ay M1bx M1by M2bx M2by M3bx M3by

deg2rad = pi/180; %convert degrees to rads
rad2deg = 180/pi; %convert rads to degrees
inch2mm = 25.4;

%% Program options
PlotLinks = 1;
PlotWorkspace = 0;
executePath = 1;

if (PlotWorkspace==1||executePath~=0)
    PlotLinks = 0;
end
%% Input variables
Cx = 0; % in mm 
Cy = 0; % in mm
Alpha = 0; % angle of platform relative to base

%% Fixed variables
SA = 170; % in mm
L = 130; % in mm
r_plt = 130; % in mm
r_base = 290; % in mm

%% Assign CB as coodinate orgin
Bx = 0; %base center
By = 0; %base center
PB1x = -r_base*cos(30*deg2rad);
PB1y = -r_base*sin(30*deg2rad);
PB2x = r_base*cos(30*deg2rad);
PB2y = -r_base*sin(30*deg2rad);
PB3x = 0;
PB3y = r_base;

%% Derived variables

PP1x = Cx-r_plt*cos((30+Alpha)*deg2rad);
PP1y = Cy-r_plt*sin((30+Alpha)*deg2rad);
PP2x = Cx+r_plt*cos((30-Alpha)*deg2rad);
PP2y = Cy-r_plt*sin((30-Alpha)*deg2rad);
PP3x = Cx-r_plt*sin(Alpha*deg2rad);
PP3y = Cy+r_plt*cos(Alpha*deg2rad);

theta1 = atan2((PP1y-PB1y),(PP1x-PB1x))*rad2deg;
theta2 = 60-(atan2((PP2y-PB2y),(PB2x-PP2x))*rad2deg)+120;
theta3 = 30-(atan2((PB3x-PP3x),(PB3y-PP3y))*rad2deg)+240;

d1 = +sqrt((PP1y-PB1y)^2+(PP1x-PB1x)^2);
d2 = +sqrt((PP2y-PB2y)^2+(PP2x-PB2x)^2);
d3 = +sqrt((PP3y-PB3y)^2+(PP3x-PB3x)^2);

dmax = SA+L;
dmin = SA-L;

%% Solve Inverse Kinematics
if PlotWorkspace ~= 1
    if (d1<=dmax && d1>=dmin && d2<=dmax && d2>=dmin && d3<=dmax && d3>=dmin)
cos_beta1 = ((d1^2)+SA^2-(L^2))/(2*d1*SA); %from cosine law L^2=d1^2+SA^2-2*d1*SA*cos_beta;
sin_beta1 = +sqrt(1-(cos_beta1^2)); % based on sinx^2+cosx^2=1;
beta1 = atan2(sin_beta1,cos_beta1)*rad2deg;
gamma1a = theta1+beta1;
gamma1b = theta1-beta1;
M1ax = PB1x+SA*cos(gamma1a*deg2rad);
M1ay = PB1y+SA*sin(gamma1a*deg2rad);
M1bx = PB1x+SA*cos(gamma1b*deg2rad);
M1by = PB1y+SA*sin(gamma1b*deg2rad);

cos_beta2 = ((d2^2)+SA^2-(L^2))/(2*d2*SA); %from cosine law L^2=d1^2+SA^2-2*d1*SA*cos_beta;
sin_beta2 = +sqrt(1-(cos_beta2^2)); % based on sinx^2+cosx^2=1;
beta2 = atan2(sin_beta2,cos_beta2)*rad2deg;
gamma2a = theta2+beta2;
gamma2b = theta2-beta2;
M2ax = PB2x+SA*cos(gamma2a*deg2rad);
M2ay = PB2y+SA*sin(gamma2a*deg2rad);
M2bx = PB2x+SA*cos(gamma2b*deg2rad);
M2by = PB2y+SA*sin(gamma2b*deg2rad);

cos_beta3 = ((d3^2)+SA^2-(L^2))/(2*d3*SA); %from cosine law L^2=d1^2+SA^2-2*d1*SA*cos_beta;
sin_beta3 = +sqrt(1-(cos_beta3^2)); % based on sinx^2+cosx^2=1;
beta3 = atan2(sin_beta3,cos_beta3)*rad2deg;
gamma3a = theta3+beta3;
gamma3b = theta3-beta3;
M3ax = PB3x+SA*cos(gamma3a*deg2rad);
M3ay = PB3y+SA*sin(gamma3a*deg2rad);
M3bx = PB3x+SA*cos(gamma3b*deg2rad);
M3by = PB3y+SA*sin(gamma3b*deg2rad);

%Verification Process 
%verify if calculatd length of the side of platform are equal with others
plt_side1 = +sqrt((PP1y-PP2y)^2+(PP1x-PP2x)^2); 
plt_side2 = +sqrt((PP2y-PP3y)^2+(PP2x-PP3x)^2); 
plt_side3 = +sqrt((PP3y-PP1y)^2+(PP3x-PP1x)^2);
%verify if calculated L1 is 130
L1a = +sqrt((PP1y-M1ay)^2+(PP1x-M1ax)^2); 
L1b = +sqrt((PP1y-M1by)^2+(PP1x-M1bx)^2); 
L2a = +sqrt((PP2y-M2ay)^2+(PP2x-M2ax)^2); 
L2b = +sqrt((PP2y-M2by)^2+(PP2x-M2bx)^2); 
L3a = +sqrt((PP3y-M3ay)^2+(PP3x-M3ax)^2);
L3b = +sqrt((PP3y-M3by)^2+(PP3x-M3bx)^2); 
    end
end

%Plotting
pltx = [PP1x,PP2x,PP3x];
plty = [PP1y,PP2y,PP3y];
basex = [PB1x,PB2x,PB3x];
basey = [PB1y,PB2y,PB3y];
PB_x = [PB1x,PB2x,PB3x,PB1x];
PB_y = [PB1y,PB2y,PB3y,PB1y];
PP_x = [PP1x,PP2x,PP3x,PP1x];
PP_y = [PP1y,PP2y,PP3y,PP1y];
D1_x = [PB1x,PP1x];
D1_y = [PB1y,PP1y];
D2_x = [PB2x,PP2x];
D2_y = [PB2y,PP2y];
D3_x = [PB3x,PP3x];
D3_y = [PB3y,PP3y];
M1a_x = [PB1x,M1ax,PP1x];
M1a_y = [PB1y,M1ay,PP1y];
M1b_x = [PB1x,M1bx,PP1x];
M1b_y = [PB1y,M1by,PP1y];
M2a_x = [PB2x,M2ax,PP2x];
M2a_y = [PB2y,M2ay,PP2y];
M2b_x = [PB2x,M2bx,PP2x];
M2b_y = [PB2y,M2by,PP2y];
M3a_x = [PB3x,M3ax,PP3x];
M3a_y = [PB3y,M3ay,PP3y];
M3b_x = [PB3x,M3bx,PP3x];
M3b_y = [PB3y,M3by,PP3y];

hold on
grid on;
title ('Parallel Robot Simulation');
ylabel('Y(mm)'),xlabel('X(mm)')
axis([-300 300 -250 350]);

%base = fill(basex,basey,[0.1,0.5,1]);
%set(base,'facealpha',0.9);
%platform = fill(pltx,plty,[0.9,0.4,0]);
%set(platform,'facealpha',0.9);

plot(PB_x,PB_y,'Color','b','LineWidth',2);

if PlotLinks == 1
plot1 = plot(PP_x,PP_y,'LineWidth',2);
plot2 = plot(D1_x,D1_y,'--','Color','k','LineWidth',1);
plot3 = plot(D2_x,D2_y,'--','Color','k','LineWidth',1);
plot4 = plot(D3_x,D3_y,'--','Color','k','LineWidth',1);
plot5 = plot(M1a_x,M1a_y,'Color','r','LineWidth',2);
plot6 = plot(M1b_x,M1b_y,'Color','r','LineWidth',2);
plot7 = plot(M2a_x,M2a_y,'Color','r','LineWidth',2);
plot8 = plot(M2b_x,M2b_y,'Color','r','LineWidth',2);
plot9 = plot(M3a_x,M3a_y,'Color','r','LineWidth',2);
plot10 = plot(M3b_x,M3b_y,'Color','r','LineWidth',2);
plot11 = plot(Bx,By,'.','markersize',20,'Color','c');
plot12 = plot(Cx,Cy,'.','markersize',20,'Color','y');
plot13 = plot(PP1x,PP1y,'.','Color','g','markersize',10);
plot14 = plot(PP2x,PP2y,'.','Color','g','markersize',10);
plot15 = plot(PP3x,PP3y,'.','Color','g','markersize',10);
plot16 = plot(PB1x,PB1y,'.','Color','g','markersize',10);
plot17 = plot(PB2x,PB2y,'.','Color','g','markersize',10);
plot18 = plot(PB3x,PB3y,'.','Color','g','markersize',10);
plot19 = plot(M1ax,M1ay,'.','Color','g','markersize',10);
plot20 = plot(M1bx,M1by,'.','Color','g','markersize',10);
plot21 = plot(M2ax,M2ay,'.','Color','g','markersize',10);
plot22 = plot(M2bx,M2by,'.','Color','g','markersize',10);
plot23 = plot(M3ax,M3ay,'.','Color','g','markersize',10);
plot24 = plot(M3bx,M3by,'.','Color','g','markersize',10);

plot25 = text(Bx,By,'  B  ');
plot26 = text(Cx,Cy,'  C  ');
plot27 = text(PP1x,PP1y,'  PP1  ');
plot28 = text(PP2x,PP2y,'  PP2  ');
plot29 = text(PP3x,PP3y,'  PP3  ');
plot30 = text(PB1x,PB1y,'  PB1  ');
plot31 = text(PB2x,PB2y,'  PB2  ');
plot32 = text(PB3x,PB3y,'  PB3  ');
plot33 = text(M1ax,M1ay,'  M1a  ');
plot34 = text(M1bx,M1by,'  M1b  ');
plot35 = text(M2ax,M2ay,'  M2a  ');
plot36 = text(M2bx,M2by,'  M2b  ');
plot37 = text(M3ax,M3ay,'  M2a  ');
plot38 = text(M3bx,M3by,'  M2b  ');

fprintf('Inputs:\tCx=%.2f\tCy=%.2f\talpha=%.2f\n',Cx,Cy,Alpha)
fprintf('--------------------------------------------------------------------------------------------\n')
fprintf('Solution 1:\tgmma1=%.2f\tM1_x=%.2f\tM1_y=%.2f\n',gamma1a,M1ax,M1ay)
fprintf('Solution 1:\tgmma2=%.2f\tM2_x=%.2f\tM2_y=%.2f\n',gamma2a,M2ax,M2ay)
fprintf('Solution 1:\tgmma3=%.2f\tM3_x=%.2f\tM3_y=%.2f\n',gamma3a,M3ax,M3ay)
fprintf('Solution 2:\tgmma1=%.2f\tM1_x=%.2f\tM1_y=%.2f\n',gamma1b,M1ax,M1by)
fprintf('Solution 2:\tgmma2=%.2f\tM2_x=%.2f\tM2_y=%.2f\n',gamma2b,M2ax,M2by)
fprintf('Solution 2:\tgmma3=%.2f\tM3_x=%.2f\tM3_y=%.2f\n',gamma3b,M3ax,M3by)
else
end


if PlotWorkspace == 1
Spacex = [ ] ;
Spacey = [ ] ;
%Alpha = 10 ;
fprintf('Inputs:\talpha = %.0f\n',Alpha)
text(50,300,'Angle of Rotation: ','Color','red','FontSize',14);
text(270,300,num2str(Alpha),'Color','red','FontSize',14);

for Cx =-200:1:200,
for Cy =-200:1:200,

PP1x = Cx-r_plt*cos((30+Alpha)*deg2rad);
PP1y = Cy-r_plt*sin((30+Alpha)*deg2rad);
PP2x = Cx+r_plt*cos((30-Alpha)*deg2rad);
PP2y = Cy-r_plt*sin((30-Alpha)*deg2rad);
PP3x = Cx-r_plt*sin(Alpha*deg2rad);
PP3y = Cy+r_plt*cos(Alpha*deg2rad);

theta1 = atan2((PP1y-PB1y),(PP1x-PB1x))*rad2deg;
theta2 = 60-(atan2((PP2y-PB2y),(PB2x-PP2x))*rad2deg)+120;
theta3 = 30-(atan2((PB3x-PP3x),(PB3y-PP3y))*rad2deg)+240;

d1 = +sqrt((PP1y-PB1y)^2+(PP1x-PB1x)^2);
d2 = +sqrt((PP2y-PB2y)^2+(PP2x-PB2x)^2);
d3 = +sqrt((PP3y-PB3y)^2+(PP3x-PB3x)^2);

if (d1<=dmax && d1>=dmin && d2<=dmax && d2>=dmin && d3<=dmax && d3>=dmin)

Spacex(end+1) = Cx;
Spacey(end+1) = Cy;

end
end
end

scatter(Spacex,Spacey,20,[0,1,0],'filled');
hold on
end


if executePath == 1

stage_no=500;
t = linspace(0,20*pi,stage_no);
Cx = 50*cos(t);
Cy = 50*sin(t);
%Cx=linspace(-50,50,stage_no);   
%Cy=linspace(-100,100,stage_no);
Alpha=linspace(-45,45,stage_no);
time=1;

total_stage=numel(Cx);
stage=1;

while stage < total_stage+1
    

    PP1x = Cx(stage)-r_plt*cos((30+Alpha(stage))*deg2rad);
    PP1y = Cy(stage)-r_plt*sin((30+Alpha(stage))*deg2rad);
    PP2x = Cx(stage)+r_plt*cos((30-Alpha(stage))*deg2rad);
    PP2y = Cy(stage)-r_plt*sin((30-Alpha(stage))*deg2rad);
    PP3x = Cx(stage)-r_plt*sin(Alpha(stage)*deg2rad);
    PP3y = Cy(stage)+r_plt*cos(Alpha(stage)*deg2rad);

    theta1 = atan2((PP1y-PB1y),(PP1x-PB1x))*rad2deg;
    theta2 = 60-(atan2((PP2y-PB2y),(PB2x-PP2x))*rad2deg)+120;
    theta3 = 30-(atan2((PB3x-PP3x),(PB3y-PP3y))*rad2deg)+240;

    d1 = +sqrt((PP1y-PB1y)^2+(PP1x-PB1x)^2);
    d2 = +sqrt((PP2y-PB2y)^2+(PP2x-PB2x)^2);
    d3 = +sqrt((PP3y-PB3y)^2+(PP3x-PB3x)^2);
    
    cos_beta1 = ((d1^2)+SA^2-(L^2))/(2*d1*SA); %from cosine law L^2=d1^2+SA^2-2*d1*SA*cos_beta;
    sin_beta1 = +sqrt(1-(cos_beta1^2)); % based on sinx^2+cosx^2=1;
    beta1 = atan2(sin_beta1,cos_beta1)*rad2deg;
    gamma1a = theta1+beta1;
    gamma1b = theta1-beta1;
    M1ax = PB1x+SA*cos(gamma1a*deg2rad);
    M1ay = PB1y+SA*sin(gamma1a*deg2rad);
    M1bx = PB1x+SA*cos(gamma1b*deg2rad);
    M1by = PB1y+SA*sin(gamma1b*deg2rad);

    cos_beta2 = ((d2^2)+SA^2-(L^2))/(2*d2*SA); %from cosine law L^2=d1^2+SA^2-2*d1*SA*cos_beta;
    sin_beta2 = +sqrt(1-(cos_beta2^2)); % based on sinx^2+cosx^2=1;
    beta2 = atan2(sin_beta2,cos_beta2)*rad2deg;
    gamma2a = theta2+beta2;
    gamma2b = theta2-beta2;
    M2ax = PB2x+SA*cos(gamma2a*deg2rad);
    M2ay = PB2y+SA*sin(gamma2a*deg2rad);
    M2bx = PB2x+SA*cos(gamma2b*deg2rad);
    M2by = PB2y+SA*sin(gamma2b*deg2rad);

    cos_beta3 = ((d3^2)+SA^2-(L^2))/(2*d3*SA); %from cosine law L^2=d1^2+SA^2-2*d1*SA*cos_beta;
    sin_beta3 = +sqrt(1-(cos_beta3^2)); % based on sinx^2+cosx^2=1;
    beta3 = atan2(sin_beta3,cos_beta3)*rad2deg;
    gamma3a = theta3+beta3;
    gamma3b = theta3-beta3;
    M3ax = PB3x+SA*cos(gamma3a*deg2rad);
    M3ay = PB3y+SA*sin(gamma3a*deg2rad);
    M3bx = PB3x+SA*cos(gamma3b*deg2rad);
    M3by = PB3y+SA*sin(gamma3b*deg2rad);
    
    %Plotting
    pltx = [PP1x,PP2x,PP3x];
    plty = [PP1y,PP2y,PP3y];
    basex = [PB1x,PB2x,PB3x];
    basey = [PB1y,PB2y,PB3y];
    PB_x = [PB1x,PB2x,PB3x,PB1x];
    PB_y = [PB1y,PB2y,PB3y,PB1y];
    PP_x = [PP1x,PP2x,PP3x,PP1x];
    PP_y = [PP1y,PP2y,PP3y,PP1y];
    D1_x = [PB1x,PP1x];
    D1_y = [PB1y,PP1y];
    D2_x = [PB2x,PP2x];
    D2_y = [PB2y,PP2y];
    D3_x = [PB3x,PP3x];
    D3_y = [PB3y,PP3y];
    M1a_x = [PB1x,M1ax,PP1x];
    M1a_y = [PB1y,M1ay,PP1y];
    M1b_x = [PB1x,M1bx,PP1x];
    M1b_y = [PB1y,M1by,PP1y];
    M2a_x = [PB2x,M2ax,PP2x];
    M2a_y = [PB2y,M2ay,PP2y];
    M2b_x = [PB2x,M2bx,PP2x];
    M2b_y = [PB2y,M2by,PP2y];
    M3a_x = [PB3x,M3ax,PP3x];
    M3a_y = [PB3y,M3ay,PP3y];
    M3b_x = [PB3x,M3bx,PP3x];
    M3b_y = [PB3y,M3by,PP3y];

    plot(PB_x,PB_y,'Color','b','LineWidth',2); %static

    plot1 = plot(PP_x,PP_y,'LineWidth',2);
    plot2 = plot(D1_x,D1_y,'--','Color','k','LineWidth',1);
    plot3 = plot(D2_x,D2_y,'--','Color','k','LineWidth',1);
    plot4 = plot(D3_x,D3_y,'--','Color','k','LineWidth',1);
    plot5 = plot(M1a_x,M1a_y,'Color','r','LineWidth',2);
    plot6 = plot(M1b_x,M1b_y,'Color','r','LineWidth',2);
    plot7 = plot(M2a_x,M2a_y,'Color','r','LineWidth',2);
    plot8 = plot(M2b_x,M2b_y,'Color','r','LineWidth',2);
    plot9 = plot(M3a_x,M3a_y,'Color','r','LineWidth',2);
    plot10 = plot(M3b_x,M3b_y,'Color','r','LineWidth',2);
    plot11 = plot(Bx,By,'.','markersize',20,'Color','c');
    plot12 = plot(Cx(stage),Cy(stage),'.','markersize',20,'Color','y');
    plot13 = plot(PP1x,PP1y,'.','Color','g','markersize',10);
    plot14 = plot(PP2x,PP2y,'.','Color','g','markersize',10);
    plot15 = plot(PP3x,PP3y,'.','Color','g','markersize',10);
    plot16 = plot(PB1x,PB1y,'.','Color','g','markersize',10);
    plot17 = plot(PB2x,PB2y,'.','Color','g','markersize',10);
    plot18 = plot(PB3x,PB3y,'.','Color','g','markersize',10);
    plot19 = plot(M1ax,M1ay,'.','Color','g','markersize',10);
    plot20 = plot(M1bx,M1by,'.','Color','g','markersize',10);
    plot21 = plot(M2ax,M2ay,'.','Color','g','markersize',10);
    plot22 = plot(M2bx,M2by,'.','Color','g','markersize',10);
    plot23 = plot(M3ax,M3ay,'.','Color','g','markersize',10);
    plot24 = plot(M3bx,M3by,'.','Color','g','markersize',10);

    plot25 = text(Bx,By,'  B  ');
    plot26 = text(Cx(stage),Cy(stage),'  C  ');
    plot27 = text(PP1x,PP1y,'  PP1  ');
    plot28 = text(PP2x,PP2y,'  PP2  ');
    plot29 = text(PP3x,PP3y,'  PP3  ');
    plot30 = text(PB1x,PB1y,'  PB1  ');
    plot31 = text(PB2x,PB2y,'  PB2  ');
    plot32 = text(PB3x,PB3y,'  PB3  ');
    plot33 = text(M1ax,M1ay,'  M1a  ');
    plot34 = text(M1bx,M1by,'  M1b  ');
    plot35 = text(M2ax,M2ay,'  M2a  ');
    plot36 = text(M2bx,M2by,'  M2b  ');
    plot37 = text(M3ax,M3ay,'  M2a  ');
    plot38 = text(M3bx,M3by,'  M2b  ');
    
    drawnow

    stage=stage+1;
    delay(time/(total_stage-1));

    if stage==total_stage+1
    
    else
    delete(plot1);
    delete(plot2);
    delete(plot3);
    delete(plot4);
    delete(plot5);
    delete(plot6);
    delete(plot7);
    delete(plot8);
    delete(plot9);
    delete(plot10);
    delete(plot11);
    delete(plot12);
    delete(plot13);
    delete(plot14);
    delete(plot15);
    delete(plot16);
    delete(plot17);
    delete(plot18);
    delete(plot19);
    delete(plot20);
    delete(plot21);
    delete(plot22);
    delete(plot23);
    delete(plot24);
    delete(plot25);
    delete(plot26);
    delete(plot27);
    delete(plot28);
    delete(plot29);
    delete(plot30);
    delete(plot31);
    delete(plot32);
    delete(plot33);
    delete(plot34);
    delete(plot35);
    delete(plot36);
    delete(plot37);
    delete(plot38); 
    end
end
end




