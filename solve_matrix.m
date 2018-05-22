clear all
syms q1 q2 q3 q4 q5 L1 L2 L3 L4 L5

T01=[cos(q1),0,sin(q1),0;sin(q1),0,-cos(q1),0;0,1,0,L1;0,0,0,1]
T12=[cos(q2),-sin(q2),0,L2*cos(q2);sin(q2),cos(q2),0,L2*sin(q2);0,0,1,0;0,0,0,1]
T23=[cos(q3),-sin(q3),0,L3*cos(q3);sin(q3),cos(q3),0,L3*sin(q3);0,0,1,0;0,0,0,1]
T34=[cos(q4),0,-sin(q4),0;sin(q4),0,cos(q4),0;0,1,0,0;0,0,0,1]
T45=[cos(q5),-sin(q5),0,0;sin(q5),cos(q5),0,0;0,0,1,L4+L5;0,0,0,1]
T02=T01*T12
T03=T02*T23
T04=T03*T34
T05=T04*T45