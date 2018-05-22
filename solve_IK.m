function [theta1_IK,theta2a_IK,theta3a_IK,theta4a_IK,theta2b_IK,theta3b_IK,theta4b_IK] = solve_IK (xTarget,yTarget,zTarget,Phi)
global rad2deg deg2rad L1 L2 L3 L4 L5 z6 x6 z4 x4 s alpha cos_beta sin_beta beta theta1_IK theta2a_IK theta3a_IK theta4a_IK theta2b_IK theta3b_IK theta4b_IK

theta1_IK = atan2(yTarget,xTarget)*rad2deg;

if theta1_IK>90&&theta1_IK<=180;
    theta1_IK=theta1_IK-180; %transform the answer to be within available angle arange
elseif theta1_IK>=-180&&theta1_IK<-90;
    theta1_IK=theta1_IK+180;
else
    theta1_IK=theta1_IK;
end

z6 = zTarget-L1; %joint 2 as origin, joint 0 is L1 distance below origin along Z-aixs
x6 = xTarget/(cos(theta1_IK*deg2rad)); %find the distance on rotated X-axis
z4 = z6-((L4+L5)*sin(Phi*deg2rad));
x4 = x6-((L4+L5)*cos(Phi*deg2rad));
s = +sqrt((z4^2)+(x4^2)); %st9raight line between joint 2 and joint 4
alpha = atan2(z4,x4)*rad2deg; %angle between s and X-aixs
cos_beta = ((L2^2)+s^2-(L3^2))/(2*L2*s); %from cosine law L3^2=L2^2+s^2-2*L2*S*cos_beta
sin_beta = +sqrt(1-(cos_beta^2)); % based on sinx^2+cosx^2=1
beta = atan2(sin_beta,cos_beta)*rad2deg;
theta2a_IK = alpha+beta; %upper position, larger angle
theta2b_IK = alpha-beta; %lower position, smaller angle
sin_theta3 = sin_beta*s/L3; %both angle shares a common opposite side m, sin_theta3=m/L3, sin_beta=m/s
cos_theta3 = +sqrt(1-(sin_theta3^2)); %based on sinx^2+cosx^2=1
theta3a_IK = -atan2(sin_theta3,cos_theta3)*rad2deg; %upper position, L3 moves downwards, negative
theta3b_IK = atan2(sin_theta3,cos_theta3)*rad2deg; %lower position, L3 moves upwards, positive
theta4a_IK = Phi-theta2a_IK-theta3a_IK;
theta4b_IK = Phi-theta2b_IK-theta3b_IK;
end