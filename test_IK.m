clc; clear;

d1 = 13.9; % Height of base to joint 2
a2 = 10.264; % Length of link 2
a3 = 10.264; % Length of link 3
a4 = 7.11; % Length of link 4 (to wrist center/end-effector)

% Define a physically reachable test target pose
target_x = 5; 
target_y = 5;   
target_z = 5;  
target_phi = -pi/2; % Jaws pointing straight down (-90 degrees)


fprintf('Testing IK for Target Pose: X=%.2f, Y=%.2f, Z=%.2f, Phi=%.2f rad\n', ...
    target_x, target_y, target_z, target_phi);
fprintf('-------------------------------------------------------------\n');

% Call the IK function
ik_solutions = findJointAngles(target_x, target_y, target_z, target_phi);

% Display and Verify Solutions
num_solutions = size(ik_solutions, 1);

if num_solutions == 0
    fprintf('No valid IK solutions found. The point might be out of reach.\n');
else
    fprintf('Found %d valid IK solutions:\n\n', num_solutions);

    for i = 1:num_solutions
        th = ik_solutions(i, :)
        fk_sol = getEndEffectorState2(th(1), th(2), th(3), th(4))

        % Print the joint angles (converted to degrees for readability)
        fprintf('Solution %d (Degrees):\n', i);
        fprintf('  Theta1: %7.2f°\n', rad2deg(th(1)));
        fprintf('  Theta2: %7.2f°\n', rad2deg(th(2)));
        fprintf('  Theta3: %7.2f°\n', rad2deg(th(3)));
        fprintf('  Theta4: %7.2f°\n\n', rad2deg(th(4)));

        
        % 
        % sum23 = th(2) + th(3);
        % sum234 = th(2) + th(3) + th(4);
        % 
        % x_check = cos(th(1)) * (a2*cos(th(2)) + a3*cos(sum23) + a4*cos(sum234));
        % y_check = sin(th(1)) * (a2*cos(th(2)) + a3*cos(sum23) + a4*cos(sum234));
        % z_check = d1 + a2*sin(th(2)) + a3*sin(sum23) + a4*sin(sum234);
        % phi_check = sum234;
        % 
        % % Print verification results and compute the error
        % fprintf('  --> FK Verification for Solution %d:\n', i);
        % fprintf('      X: %8.2f (Error: %.4f)\n', x_check, abs(target_x - x_check));
        % fprintf('      Y: %8.2f (Error: %.4f)\n', y_check, abs(target_y - y_check));
        % fprintf('      Z: %8.2f (Error: %.4f)\n', z_check, abs(target_z - z_check));
        % fprintf('      Phi: %6.2f (Error: %.4f)\n', phi_check, abs(target_phi - phi_check));
        % fprintf('-------------------------------------------------------------\n');
    end

    theta_selected = findSolution(ik_solutions, fk_sol(5))

    
end