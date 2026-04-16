function solutions = findJointAngles(x, y, z, phi)
    d1 = 13.9; % Height of base to joint 2
    a2 = 10.264; % Length of link 2
    a3 = 10.264; % Length of link 3
    a4 = 7.11; % Length of link 4 (to wrist center/end-effector)
    
    solutions = []; 
    
    % Step 1: Calculate the two possible solutions for theta1 
    theta1_a = atan2(y, x);
    theta1_b = atan2(-y, -x);
    theta1_opts = [theta1_a, theta1_b];
    
    % Iterate over both theta1 solutions
    for i = 1:2
        th1 = theta1_opts(i);
        
        % Calculate intermediate planar coordinates (x_bar, y_bar, z_bar)
        % We use an if-condition to avoid division by zero when cos(th1) or sin(th1) is near 0.
        if abs(cos(th1)) > abs(sin(th1))
            x_bar = (x / cos(th1)) - a4 * cos(phi);
            y_bar = x_bar; % Theoretically identical in planar projection
        else
            y_bar = (y / sin(th1)) - a4 * cos(phi);
            x_bar = y_bar;
        end
        
        z_bar = z - d1 - a4 * sin(phi);
        
        % Step 2: Calculate u = cos(theta3)
        u = (x_bar^2 + z_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);
        
        % Check if the point is physically reachable (|u| <= 1)
        if abs(u) <= 1
            % Step 3: Two possible solutions for theta3 (Elbow up / Elbow down)
            th3_a = atan2(sqrt(1 - u^2), u);
            th3_b = atan2(-sqrt(1 - u^2), u);
            theta3_opts = [th3_a, th3_b];
            
            % Iterate over both theta3 solutions
            for j = 1:2
                th3 = theta3_opts(j);
                sin_th3 = sin(th3); % Represents +/- sqrt(1-u^2)
                
                % Step 4: Calculate theta2
                denom = a2^2 + 2 * a2 * a3 * u + a3^2;
                
                cos_th2 = (y_bar * (a2 + a3 * u) + z_bar * (a3 * sin_th3)) / denom;
                sin_th2 = (z_bar * (a2 + a3 * u) - y_bar * (a3 * sin_th3)) / denom;
                
                th2 = atan2(sin_th2, cos_th2);
                
                % Step 5: Calculate theta4 based on the orientation constraint
                th4 = phi - th2 - th3;
                
                solutions = [solutions; th1, th2, th3, th4];
            end
        end
    end
end
