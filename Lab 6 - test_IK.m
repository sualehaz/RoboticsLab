clc; clear;
d1 = 13.9; 
a2 = 10.264; 
a3 = 10.264; 
a4 = 7.11;

target_x = 150; 
target_y = 50;   
target_z = 50;  
target_phi = -pi/2; 

fprintf('Testing IK for Target Pose: X=%.2f, Y=%.2f, Z=%.2f, Phi=%.2f rad\n', ...
    target_x, target_y, target_z, target_phi);
fprintf('-------------------------------------------------------------\n');

ik_solutions = findJointAngles(target_x, target_y, target_z, target_phi);

num_solutions = size(ik_solutions, 1);

if num_solutions == 0
    fprintf('No valid IK solutions found. The point might be out of reach.\n');
else
    fprintf('Found %d valid IK solutions:\n\n', num_solutions);
    
    for i = 1:num_solutions
        th = ik_solutions(i, :);
      
        fprintf('Solution %d (Degrees):\n', i);
        fprintf('  Theta1: %7.2f°\n', rad2deg(th(1)));
        fprintf('  Theta2: %7.2f°\n', rad2deg(th(2)));
        fprintf('  Theta3: %7.2f°\n', rad2deg(th(3)));
        fprintf('  Theta4: %7.2f°\n\n', rad2deg(th(4)));
        
        % --- FORWARD KINEMATICS VERIFICATION ---
        % Plugging the IK angles back into the algebraic FK equations 
        
        sum23 = th(2) + th(3);
        sum234 = th(2) + th(3) + th(4);
        
        x_check = cos(th(1)) * (a2*cos(th(2)) + a3*cos(sum23) + a4*cos(sum234));
        y_check = sin(th(1)) * (a2*cos(th(2)) + a3*cos(sum23) + a4*cos(sum234));
        z_check = d1 + a2*sin(th(2)) + a3*sin(sum23) + a4*sin(sum234);
        phi_check = sum234;
        
        fprintf('  --> FK Verification for Solution %d:\n', i);
        fprintf('      X: %8.2f (Error: %.4f)\n', x_check, abs(target_x - x_check));
        fprintf('      Y: %8.2f (Error: %.4f)\n', y_check, abs(target_y - y_check));
        fprintf('      Z: %8.2f (Error: %.4f)\n', z_check, abs(target_z - z_check));
        fprintf('      Phi: %6.2f (Error: %.4f)\n', phi_check, abs(target_phi - phi_check));
        fprintf('-------------------------------------------------------------\n');
    end
end

% -------------------------------------------------------------------------------------------%
% representation with rigid body tree %

clc; clear; close all;

d1 = 13.9; 
a2 = 10.264; 
a3 = 10.264; 
a4 = 7.11; 

a_dh = [0, a2, a3, a4];
alpha_dh = [pi/2, 0, 0, 0];
d_dh = [d1, 0, 0, 0];

robot = rigidBodyTree("MaxNumBodies",4,"DataFormat","row");
for i = 1:4
    body  = rigidBody(['link' num2str(i)]);
    joint = rigidBodyJoint(['joint' num2str(i)], 'revolute');
    
    setFixedTransform(joint, [a_dh(i) alpha_dh(i) d_dh(i) 0], 'dh');
    body.Joint = joint;
    
    if i == 1
        parentName = 'base';
    else
        parentName = ['link' num2str(i-1)];
    end
    addBody(robot, body, parentName);
end

% --- 3. DEFINE TARGET POSE ---
target_x = -15; 
target_y = 5;   
target_z = 5;  
target_phi = -pi/2; 
% --- 4. CALCULATE IK SOLUTIONS ---
ik_solutions = findJointAngles(target_x, target_y, target_z, target_phi);
num_solutions = size(ik_solutions, 1);

% --- 5. VISUALIZATION LOOP ---
if num_solutions == 0
    fprintf('No valid IK solutions found.\n');
else
    fprintf('Found %d valid solutions. Plotting...\n', num_solutions);
    
    for i = 1:num_solutions
        th = ik_solutions(i, :);
        
        figure('Name', ['IK Solution ' num2str(i)]);
        
        show(robot, th); 
        hold on;
        
        plot3(target_x, target_y, target_z, 'r*', 'MarkerSize', 10, 'LineWidth', 2);
        grid on;
        view(45, 30);
        title(['Solution ' num2str(i) ' Configuration']);
        xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
        
        fprintf('Solution %d: [%.2f, %.2f, %.2f, %.2f] rad\n', i, th(1), th(2), th(3), th(4));
    end
end
