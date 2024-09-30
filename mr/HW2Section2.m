%HW2 Section 2
L1 = 0.34;
L2 = 0.4;
L3 = 0.4;
L4 = 0.15;
thetaList2 =[0;0;0;0;0;0;0];
% Part a 
slist2 = [[0;0;-1;0;0;0]...
           [1;0;0;0;-L1;0]...
           [0;0;-1;0;0;0]...
           [1;0;0;0;-(L1+L2);0]...
           [0;0;-1;0;0;0]...
           [1;0;0;0;-(L1+L2+L3);0]...
           [0;0;-1;0;0;0]];


sJ = JacobianSpace(slist2, thetaList2);
disp("Space Jacobian @ Home Configuration:")
disp(sJ);

% Part b

blsit2 = [[1;0;0;0;(L4+L2+L3+(L1*0.5));0]...
            [0;0;-1;0;0;0]...
            [1;0;0;0;(L4+L2+L3);0]...
            [0;0;-1;0;0;0]...
            [1;0;0;0;(L4+L3);0]...
            [0;0;-1;0;0;0]...
            [1;0;0;0;L4;0]];

bJ = JacobianBody(blist2, thetaList2);
disp("Body Jacobian @ Home Configuration:")
disp(bJ);

% part c 
rS = rank(sJ);
rB = rank(bJ);
disp(['Rank of the Space Jacobians is ', num2str(rS)]);
disp(['Rank of the Body Jacobians is ', num2str(rB)]);


% part e
torque = transpose(bJ)*[1;1;1;1;1;1];
disp("torque:")
disp(torque);

