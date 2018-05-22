function UpdateArm1(x,y,z,Phi,time,plot_end)
global theta1_IK theta2a_IK theta3a_IK theta4a_IK theta2b_IK theta3b_IK theta4b_IK;
global L1 L2 L3 L4 L5 theta1 theta2 theta3 theta4 theta5 deg2rad rad2deg theta1_delta theta2_delta theta3_delta theta4_delta;
global timeslot EnableVelocityPrint
[theta1_IK,theta2a_IK,theta3a_IK,theta4a_IK,theta2b_IK,theta3b_IK,theta4b_IK] = solve_IK(x,y,z,Phi);

theta1_delta=(theta1_IK-theta1*rad2deg)/(time/timeslot);
theta2_delta=(theta2a_IK-theta2*rad2deg)/(time/timeslot);
theta3_delta=(theta3a_IK-theta3*rad2deg)/(time/timeslot);
theta4_delta=(theta4a_IK-theta4*rad2deg)/(time/timeslot);

time_left=time;

while time_left>0
  
theta1=theta1+theta1_delta*deg2rad;
theta2=theta2+theta2_delta*deg2rad;
theta3=theta3+theta3_delta*deg2rad;
theta4=theta4+theta4_delta*deg2rad;

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

fprintf('x= %.2f \t y= %.2f \t z= %.2f \t',T05(1,4),T05(2,4),T05(3,4));
fprintf('1= %.2f \t 2= %.2f \t 3= %.2f \t 4= %.2f \n',theta1*rad2deg,theta2*rad2deg,theta3*rad2deg,theta4*rad2deg);

if (EnableVelocityPrint==1||EnableVelocityPrint==2)
Jacobian_get_velocity(x,y,z,Phi,theta1_delta,theta2_delta,theta3_delta,theta4_delta,timeslot);
end

drawnow

time_left=time_left-0.1;
delay(timeslot);

if plot_end == 1 && time_left <= 0   
else
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

end

end