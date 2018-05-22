function T = get_dh_matrix_distal (parameters)
T = [(cos(parameters(1,4))),-(cos(parameters(1,2)))*(sin(parameters(1,4))),(sin(parameters(1,2)))*(sin(parameters(1,4))),(parameters(1,1)*(cos(parameters(1,4))));
(sin(parameters(1,4))),(cos(parameters(1,2)))*(cos(parameters(1,4))),-(sin(parameters(1,2)))*(cos(parameters(1,4))),(parameters(1,1)*(sin(parameters(1,4))));
0,(sin(parameters(1,2))),(cos(parameters(1,2))),(parameters(1,3));
0,0,0,1];
end