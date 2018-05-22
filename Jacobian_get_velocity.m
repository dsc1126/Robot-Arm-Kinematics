function [EE_kinematics] = Jacobian_get_velocity(Px,Py,Pz,Phi,theta1_delta,theta2_delta,theta3_delta,theta4_delta,interval)
global L1 L2 L3 L4 L5 EE_kinematics
syms p1 p2 p3 p4
global theta1 theta2 theta3 theta4 J EnableVelocityPrint rad2deg

theta1_velocity = theta1_delta/interval;
theta2_velocity = theta2_delta/interval;
theta3_velocity = theta3_delta/interval;
theta4_velocity = theta4_delta/interval;

Px = L2*cos(p1)*cos(p2)-(cos(p4)*(cos(p1)*cos(p2)*sin(p3)+cos(p1)*cos(p3)*sin(p2))+sin(p4)*(cos(p1)*cos(p2)*cos(p3)-cos(p1)*sin(p2)*sin(p3)))*(L4+L5)+L3*cos(p1)*cos(p2)*cos(p3)-L3*cos(p1)*sin(p2)*sin(p3);
Py = L2*cos(p2)*sin(p1)-(cos(p4)*(cos(p2)*sin(p1)*sin(p3)+cos(p3)*sin(p1)*sin(p2))-sin(p4)*(sin(p1)*sin(p2)*sin(p3)-cos(p2)*cos(p3)*sin(p1)))*(L4+L5)+L3*cos(p2)*cos(p3)*sin(p1)-L3*sin(p1)*sin(p2)*sin(p3);
Pz = L1+(cos(p4)*(cos(p2)*cos(p3)-sin(p2)*sin(p3))-sin(p4)*(cos(p2)*sin(p3)+cos(p3)*sin(p2)))*(L4+L5)+L2*sin(p2)+L3*cos(p2)*sin(p3)+L3*cos(p3)*sin(p2);
Phi = p1+p2+p3;
format short g
J = [diff(Px,p1),diff(Px,p2),diff(Px,p3),diff(Px,p4);
    diff(Py,p1),diff(Py,p2),diff(Py,p3),diff(Py,p4);
    diff(Pz,p1),diff(Pz,p2),diff(Pz,p3),diff(Pz,p4);
    diff(Phi,p1),diff(Phi,p2),diff(Phi,p3),diff(Phi,p4)];
p1=theta1;
p2=theta2;
p3=theta3;
p4=theta4;
eval(J);

joint_velocity = [theta1_velocity;theta2_velocity;theta3_velocity;theta4_velocity];
joint_velocity_number = double(joint_velocity);
EE_kinematics = J*joint_velocity_number;
eval(EE_kinematics);
EE_x_velocity = EE_kinematics(1,1);
EE_y_velocity = EE_kinematics(2,1);
EE_z_velocity = EE_kinematics(3,1);

fprintf('EE_x_velocity = %.2f \t',eval(EE_x_velocity));%mm/s %vpa(double(eval(EE_x_velocity)))%vpa(double(xv))
fprintf('EE_y_velocity = %.2f \t',eval(EE_y_velocity));%mm/s 
fprintf('EE_z_velocity = %.2f \t',eval(EE_z_velocity));%mm/s
fprintf('EE_Phi_velocity = %.3f \n',EE_kinematics(4,1));%deg/s

if EnableVelocityPrint == 2
fprintf('theta1_velocity = %.2f \t',theta1_velocity); %deg/s
fprintf('theta2_velocity = %.2f \t',theta2_velocity); %deg/s
fprintf('theta3_velocity = %.2f \t',theta3_velocity); %deg/s
fprintf('theta4_velocity = %.2f \n',theta4_velocity); %deg/s
end
end