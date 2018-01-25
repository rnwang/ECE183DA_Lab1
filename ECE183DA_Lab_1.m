%DH Parameters
%Constants
a_1 = 0;
a_2 = 0;
a_3 = 0.1;
a_4 = 0;
a_5 = 0;
alpha_1 = 0;
alpha_2 = -pi/2;
alpha_3 = pi/2;
alpha_4 = 0;
alpha_5 = -pi/2;
d_1 = 0;
d_2 = 1;
d_4 = 0;
d_5 = 0;
theta_3 = 0;
%Variables
syms d_3;
syms theta_1 theta_2 theta_4 theta_5;
sym_joint_vars = [d_3, theta_1, theta_2, theta_4, theta_5];
val_joint_vars = [0, 0, 0, 0, 0];
%Setup Matrices
T01 = [ cos(theta_1), -sin(theta_1), 0, a_1; ...
  cos(alpha_1)*sin(theta_1), cos(alpha_1)*cos(theta_1), -sin(alpha_1), -d_1*sin(alpha_1); ...
  sin(alpha_1)*sin(theta_1), sin(alpha_1)*cos(theta_1), cos(alpha_1), d_1*cos(alpha_1); ...
  0, 0, 0, 1];
T12 = [ cos(theta_2), -sin(theta_2), 0, a_2; ...
  cos(alpha_2)*sin(theta_2), cos(alpha_2)*cos(theta_2), -sin(alpha_2), -d_2*sin(alpha_2); ...
  sin(alpha_2)*sin(theta_2), sin(alpha_2)*cos(theta_2), cos(alpha_2), d_2*cos(alpha_2); ...
  0, 0, 0, 1];
T23 = [ cos(theta_3), -sin(theta_3), 0, a_3; ...
  cos(alpha_3)*sin(theta_3), cos(alpha_3)*cos(theta_3), -sin(alpha_3), -d_3*sin(alpha_3); ...
  sin(alpha_3)*sin(theta_3), sin(alpha_3)*cos(theta_3), cos(alpha_3), d_3*cos(alpha_3); ...
  0, 0, 0, 1];
T34 = [ cos(theta_4), -sin(theta_4), 0, a_4; ...
  cos(alpha_4)*sin(theta_4), cos(alpha_4)*cos(theta_4), -sin(alpha_4), -d_4*sin(alpha_4); ...
  sin(alpha_4)*sin(theta_4), sin(alpha_4)*cos(theta_4), cos(alpha_4), d_4*cos(alpha_4); ...
  0, 0, 0, 1];
T45 = [ cos(theta_5), -sin(theta_5), 0, a_5; ...
  cos(alpha_5)*sin(theta_5), cos(alpha_5)*cos(theta_5), -sin(alpha_5), -d_5*sin(alpha_5); ...
  sin(alpha_5)*sin(theta_5), sin(alpha_5)*cos(theta_5), cos(alpha_5), d_5*cos(alpha_5); ...
  0, 0, 0, 1];
T05 = T01*T12*T23*T34*T45;

%extract xyz coordinates and angles
xyz = [T05(1,4);T05(2,4);T05(3,4)];
%phi = atan2(T05(3,2), T05(3,3));
%temp = sqrt((T05(3, 2)^2 + T05(3, 3)^2));
%theta = atan2(-T05(3, 1), temp);
%psi = atan2(T05(2, 1), T05(1, 1));
%angles = [phi;theta;psi];

%p = vertcat(xyz,angles);
p = xyz;

%I'm not good at names ok
the_jacobian = jacobian(p, [d_3, theta_1, theta_2, theta_4, theta_5]);

%Starting point at rest position
starting = eval(subs(p, sym_joint_vars, val_joint_vars));

%Ending point with these values I made up. 
val_joint_vars = [.2, pi/3, pi/3, pi/3, pi/3];
ending = eval(subs(p, sym_joint_vars, val_joint_vars));

%Now we do the IK
current = starting;
temp_vals = val_joint_vars;
index = 1;
x1(index) = current(1);
y1(index) = current(2);
z1(index) = current(3);
while (~isIn(current(1, 1), ending(1, 1), 0.01) || ~isIn(current(2, 1), ending(2, 1), 0.01) || ~isIn(current(3, 1), ending(3, 1), 0.001) )
    step = (ending-current)/10;
    jacobian_val = eval(subs(the_jacobian, sym_joint_vars, val_joint_vars));
    dq = pinv(jacobian_val)*step;
    temp_vals = temp_vals + dq.';
    current = eval(subs(p, sym_joint_vars, temp_vals));
    index = index + 1;
    x1(index) = current(1);
    y1(index) = current(2);
    z1(index) = current(3);
    
end
%Generate straight line
x2(1) = starting(1);
y2(1) = starting(2);
z2(1) = starting(3);
x2(2) = ending(1);
y2(2) = ending(2);
z2(2) = ending(3);

%FK part
temp_vals_FK = [0, 0, 0, 0, 0];
end_config_FK = temp_vals;
current_FK = eval(subs(p, sym_joint_vars, temp_vals_FK));
x3(1) = current_FK(1, 1);
y3(1) = current_FK(2, 1);
z3(1) = current_FK(3, 1);
start_config_FK = [0; 0; 0; 0; 0];
step_FK =(1 / 10) * (end_config_FK - start_config_FK.');
for i = 2:10
    temp_vals_FK = temp_vals_FK + step_FK;
    current_FK = eval(subs(p, sym_joint_vars, temp_vals_FK));
    x3(i) = current(1);
    y3(i) = current(2);
    z3(i) = current(3);
end




%plots
hold on
plot3(x1,y1,z1,'b');
xlabel('x');
ylabel('y');
zlabel('z');
plot3(x2,y2,z2, 'r');
plot3(x3,y3,z3, 'g');
hold off

