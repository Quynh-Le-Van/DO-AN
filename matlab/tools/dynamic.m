%% Private Variable
syms theta1 theta2 theta3 theta1_d theta2_d theta3_d  l1 l2 l3 m1 m2 m3 k1 k2 k3 u1 u2 u3 k u p1 p2 p3 p1_d p2_d p3_d g L; 
syms ld_1 ld_2 ld_3 ldd_1 ldd_2 ldd_3 theta1_dd theta2_dd theta3_dd t1 t2 t3 
syms M C Gra dM dMtheta1 dMtheta2 dMtheta3;

assume(theta1, 'real');
assume(theta2, 'real');
assume(theta3, 'real');
assume(theta1_d, 'real');
assume(theta2_d, 'real');
assume(theta3_d, 'real');
assume(l1, 'real');
assume(l2, 'real');
assume(l3, 'real');
assume(m1, 'real');
assume(m2, 'real');
assume(m3, 'real');
assume(k1, 'real');
assume(k2, 'real');
assume(k3, 'real');
assume(u1, 'real');
assume(u2, 'real');
assume(u3, 'real');
assume(g, 'real');
assume(k, 'real');
assume(u, 'real');
assume(L, 'real');
assume(ld_1, 'real');
assume(ld_2, 'real');
assume(ld_3, 'real');
assume(ldd_1, 'real');
assume(ldd_2, 'real');
assume(ldd_3, 'real');
assume(theta1_dd, 'real');
assume(theta2_dd, 'real');
assume(theta3_dd, 'real');
assume(t1, 'real');
assume(t2, 'real');
assume(t3, 'real');
assume(M, 'real');
assume(C, 'real');
assume(Gra, 'real');
assume(dM, 'real');
assume(dMtheta1, 'real');
assume(dMtheta2, 'real');
assume(dMtheta3, 'real');

G = [0 0 -g];

%% ===================== MAIN ===============================

%% Joint 1st
p1 = [0; 0; l1];
p1_d = [0; 0; 0];

% Kinetic energy and Potential energy  
k1 = 0;
u1 = -m1*G*p1;

%% Joint 2nd 
p2 = [cos(theta1)*l2*cos(theta2);...
      sin(theta1)*l2*cos(theta2);...
      l1 + l2*sin(theta2)];

p2_d = [-sin(theta1)*l2*cos(theta2)*theta1_d - cos(theta1)*l2*sin(theta2)*theta2_d;...
         cos(theta1)*l2*cos(theta2)*theta1_d - sin(theta1)*l2*sin(theta2)*theta2_d;...
         l2*cos(theta2)*theta2_d];

% Kinetic energy and Potential energy
k2 = 0.5*m2*p2_d'*p2_d;
u2 = -m2*G*p2;

%% Joint 3rd
p3 = [cos(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2));...
      sin(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2));...
      l1 + l3*sin(theta2 + theta3) + l2*sin(theta2)];

p3_d = [-sin(theta1)*(l2*cos(theta2) + l3*cos(theta2 + theta3))*theta1_d + cos(theta1)*(-l2*sin(theta2) - l3*sin(theta2 + theta3))*theta2_d + cos(theta1)*(-l3*sin(theta2 + theta3))*theta3_d;...
         cos(theta1)*(l2*cos(theta2) + l3*cos(theta2 + theta3))*theta1_d + sin(theta1)*(-l2*sin(theta2) - l3*sin(theta2 + theta3))*theta2_d + sin(theta1)*(-l3*sin(theta2 + theta3))*theta3_d;...
        (l2*cos(theta2) + l3*cos(theta2 + theta3))*theta2_d + l3*cos(theta2 + theta3)*theta3_d];

% Kinetic energy and Potential energy
k3 = 0.5*m3*p3_d'*p3_d;
u3 = -m3*G*p3;

%% Total Kinetic Energy and Potential Energy
k = k1 + k2 + k3;
u = u1 + u2 + u3;

%% Lagrange equation
L = simplify(k - u);

ld_1 = diff(L, theta1);
ld_2 = diff(L, theta2);
ld_3 = diff(L, theta3);

ldd_1 = diff(L, theta1_d);
ldd_2 = diff(L, theta2_d);
ldd_3 = diff(L, theta3_d);

%% Torque
t1 = simplify(  diff(ldd_1, theta1)*theta1_d + diff(ldd_1, theta2)*theta2_d + diff(ldd_1, theta3)*theta3_d...
              + diff(ldd_1, theta1_d)*theta1_dd + diff(ldd_1, theta2_d)*theta2_dd + diff(ldd_1, theta3_d)*theta3_dd...
              - ld_1);

t2 = simplify(  diff(ldd_2, theta1)*theta1_d + diff(ldd_2, theta2)*theta2_d + diff(ldd_2, theta3)*theta3_d...
              + diff(ldd_2, theta1_d)*theta1_dd + diff(ldd_2, theta2_d)*theta2_dd + diff(ldd_2, theta3_d)*theta3_dd...
              - ld_2);

t3 = simplify(  diff(ldd_3, theta1)*theta1_d + diff(ldd_3, theta2)*theta2_d + diff(ldd_3, theta3)*theta3_d...
              + diff(ldd_3, theta1_d)*theta1_dd + diff(ldd_3, theta2_d)*theta2_dd + diff(ldd_3, theta3_d)*theta3_dd...
              - ld_3);


%% Inertia matrix
M = simplify([diff(t1, theta1_dd), diff(t1, theta2_dd), diff(t1, theta3_dd);...
              diff(t2, theta1_dd), diff(t2, theta2_dd), diff(t2, theta3_dd);...
              diff(t3, theta1_dd), diff(t3, theta2_dd), diff(t3, theta3_dd)]);

%% Gravity vector
Gra = simplify([diff(u, theta1);...
                diff(u, theta2);...
                diff(u, theta3)]);

%% Coriolis matrix
dM = simplify([diff(M(1,1), theta1)*theta1_d + diff(M(1,1), theta2)*theta2_d + diff(M(1,1), theta3)*theta3_d, diff(M(1,2), theta1)*theta1_d + diff(M(1,2), theta2)*theta2_d + diff(M(1,2), theta3)*theta3_d, diff(M(1,3), theta1)*theta1_d + diff(M(1,3), theta2)*theta2_d + diff(M(1,3), theta3)*theta3_d;...
               diff(M(2,1), theta1)*theta1_d + diff(M(2,1), theta2)*theta2_d + diff(M(2,1), theta3)*theta3_d, diff(M(2,2), theta1)*theta1_d + diff(M(2,2), theta2)*theta2_d + diff(M(2,2), theta3)*theta3_d, diff(M(2,3), theta1)*theta1_d + diff(M(2,3), theta2)*theta2_d + diff(M(2,3), theta3)*theta3_d;...   
               diff(M(3,1), theta1)*theta1_d + diff(M(3,1), theta2)*theta2_d + diff(M(3,1), theta3)*theta3_d, diff(M(3,2), theta1)*theta1_d + diff(M(3,2), theta2)*theta2_d + diff(M(3,2), theta3)*theta3_d, diff(M(3,3), theta1)*theta1_d + diff(M(3,3), theta2)*theta2_d + diff(M(3,3), theta3)*theta3_d   ]);

dMtheta1 = Com_dMdthetai(M, theta1);
dMtheta2 = Com_dMdthetai(M, theta2);
dMtheta3 = Com_dMdthetai(M, theta3);

t = [t1, t2, t3]';
dtheta = [theta1_d, theta2_d, theta3_d]';
ddtheta = [theta1_dd, theta2_dd theta3_dd]';

C = simplify(dM - 0.5*([dtheta'*dMtheta1; dtheta'*dMtheta2; dtheta'*dMtheta3]));

%% Public function define 
function dMdthetai = Com_dMdthetai(M, thetai)
      dMdthetai = simplify([diff(M(1,1), thetai), diff(M(1,2), thetai), diff(M(1,3), thetai);...
                            diff(M(2,1), thetai), diff(M(2,2), thetai), diff(M(2,3), thetai);...
                            diff(M(3,1), thetai), diff(M(3,2), thetai), diff(M(3,3), thetai)]);
end   


% %% Init
% a=0; % distance to interest point in meters (m)
% b=0.2; % height 
% c=0;
% l1=0.25;
% l2=0.25;
% l3=0.1;

% %% Jacobian Matrix
% J11 = cos(phi);
% J12 = -sin(phi);
% J13 = -a*sin(phi)-l1*cos(theta2)*sin(phi+theta1)-l2*cos(theta2+theta3)*sin(phi+theta1) - l3*sin(phi+theta1);
% J14 = -l1*cos(theta2)*sin(phi+theta1)-l2*cos(theta2+theta3)*sin(phi+theta1) - l3*sin(phi+theta1);
% J15 = -l1*sin(theta2)*cos(phi+theta1)-l2*sin(theta2+theta3)*cos(phi+theta1);
% J16 = -l2*sin(theta2+theta3)*cos(phi+theta1);

% J21 = sin(phi);
% J22 = cos(phi);
% J23 = a*cos(phi)+l1*cos(theta2)*cos(phi+theta1)+l2*cos(theta2+theta3)*cos(phi+theta1) + l3*cos(phi+theta1);
% J24 = l1*cos(theta2)*cos(phi+theta1)+l2*cos(theta2+theta3)*cos(phi+theta1) + l3*cos(phi+theta1);
% J25 = -l1*sin(theta2)*sin(phi+theta1)-l2*sin(theta2+theta3)*sin(phi+theta1);
% J26 = -l2*sin(theta2+theta3)*sin(phi+theta1);

% J31 = 0;
% J32 = 0;
% J33 = 0;
% J34 = 0;
% J35 = l1*cos(theta2)+l2*cos(theta2+theta3);
% J36 = l2*cos(theta2+theta3);

% J=[J11 J12 J13 J14 J15 J16;...
%       J21 J22 J23 J24 J25 J26;...
%       J31 J32 J33 J34 J35 J36];
      


J11 = -l1*cos(theta2)*sin(theta1)-l2*cos(theta2+theta3)*sin(theta1) - l3*sin(theta1);
J12 = -l1*sin(theta2)*cos(theta1)-l2*sin(theta2+theta3)*cos(theta1);
J13 = -l2*sin(theta2+theta3)*cos(theta1);

J21 = l1*cos(theta2)*cos(theta1)+l2*cos(theta2+theta3)*cos(theta1) + l3*cos(theta1);
J22 = -l1*sin(theta2)*sin(theta1)-l2*sin(theta2+theta3)*sin(theta1);
J23 = -l2*sin(theta2+theta3)*sin(theta1);

J34 = 0;
J35 = l1*cos(theta2)+l2*cos(theta2+theta3);
J36 = l2*cos(theta2+theta3);