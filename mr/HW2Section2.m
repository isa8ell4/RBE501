%HW2 Section 2
L1 = 0.34;
L2 = 0.4;
L3 = 0.4;
L4 = 0.15;
thetaList2 =[0;0;0;0;0;0;0];
% Part a 
slist2 = [[0;0;1;0;0;0]...
           [1;0;0;0;L1;0]...
           [0;0;1;0;0;0]...
           [1;0;0;0;(L1+L2);0]...
           [0;0;1;0;0;0]...
           [1;0;0;0;(L1+L2+L3);0]...
           [0;0;1;0;0;0]];


sJ = JacobianSpace(slist2, thetaList2);
disp("Space Jacobian @ Home Configuration:")
disp(sJ);

% Part b
          
blist2 = [[0;0;1;0;0;0]...
            [1;0;0;0;-(L4+L2+L3);0]...
            [0;0;1;0;0;0]...
            [1;0;0;0;-(L4+L3);0]...
            [0;0;1;0;0;0]...
            [1;0;0;0;-L4;0]...
            [0;0;1;0;0;0]];

bJ = JacobianBody(blist2, thetaList2);
disp("Body Jacobian @ Home Configuration:")
disp(bJ);

% part c 
rS = rank(sJ);
rB = rank(bJ);
disp(['Rank of the Space Jacobians is ', num2str(rS)]);
disp(['Rank of the Body Jacobians is ', num2str(rB)]);


% part e
% all thetas need to be 30degrees
thetaList2e = [deg2rad(30);deg2rad(30);deg2rad(30);deg2rad(30);deg2rad(30);deg2rad(30);deg2rad(30)];
bJ3 = JacobianBody(blist2,thetaList2e);
torque = transpose(bJ3)*[1;1;1;1;1;1];
disp("torque:")
disp(torque);

