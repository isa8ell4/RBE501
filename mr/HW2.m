% HW2 

% Section 1a
Slist1 = [[-1; 0;  0;  0; -10;    -25],... 
       [0; -1;  0;  -20; 0;    0],...
       [1; 0; 0; 0; -45; (25+30*tan(deg2rad(30)))]];
M1 = [[1, 0, 0, 0]; 
    [0, 0, -1, -(15+30*tan(deg2rad(30)))]; 
    [0, 1, 0, -55]; 
    [0, 0, 0, 1]];
thetalist1a =[deg2rad(60); deg2rad(90); 0];

disp("Section 1a");
disp("T = ")
disp(FKinSpace(M1, Slist1, thetalist1a));
% disp(FkPoe(Slist1, M1, thetalist1a));

% Section 1b
disp("Section 1b")
T1b0 = [[1, 0, 0, 0]; 
    [0, 0, -1, -(15+30*tan(deg2rad(30)))]; 
    [0, 1, 0, -55]; 
    [0, 0, 0, 1]];

thetalist01b0 = [deg2rad(5); deg2rad(5); 0];

eomg = 0.0001;
ev = 0.00001;

disp("thetaList for T1:")
disp(IKinSpace(Slist1, M1, T1b0, thetalist01b0, eomg, ev));

disp("thetaList for T2:");
T1b2 = [[0.7071, -0.5, -0.5, 27.6777]; 
    [0.7071, 0.5, 0.5, -82.6777]; 
    [0, -0.7071, 0.7071, 27.32]; 
    [0, 0, 0, 1]];
thetalist01b2 = [deg2rad(90);deg2rad(35);deg2rad(-45)];
disp(IKinSpace(Slist1, M1, T1b2, thetalist01b2, eomg, ev));

disp("verification for T2:")
thetalist01b1_verification = [1.5708; 0.7854; -0.7854];
disp(FKinSpace(M1, Slist1, thetalist01b1_verification));


T3= [[0.866, 0.25, -0.433, -10.6699];
     [-0.433, 0.8080, -0.3995, -65.1557];
     [0.25, 0.5335, 0.8080, 37.4098;]
     [0, 0, 0, 1]];

thetaList1b3 = [deg2rad(120); deg2rad(-30); deg2rad(60)];
% disp("guessing")
% disp(FKinSpace(M1, Slist1, thetaList1b2))
disp("thetaList for T3:");
fkTL1b3 = IKinSpace(Slist1, M1, T3, thetaList1b3, eomg, ev);
disp(fkTL1b3);
disp("verification for T3:")
disp(FKinSpace(M1, Slist1, fkTL1b3));

