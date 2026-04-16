function isColliding = checkSelfCollision(robot, theta_initial, theta_final)
    
    % 2. Define the number of intermediate points to check
    numSamples = 50; 
    
    % 3. Generate weights for the open interval (0, 1)
    % We use numSamples + 2 so we can discard the 0 and 1 endpoints
    steps = linspace(0, 1, numSamples + 2);
    interp_weights = steps(2:end-1); 

    isColliding = false;

    table = collisionBox(50, 50, 2);
    table.Pose = trvec2tform([0, 0, -1]);

    % 4. Iterate through the path
    for k = 1:length(interp_weights)
        % Linearly interpolate between the two configurations
        current_theta = theta_initial + interp_weights(k) * (theta_final - theta_initial);

        
        hitTable = any(checkCollision(robot, current_theta, {table}));
    
    % 3. Evaluate the results
        if  hitTable
            isColliding = true;
        end
        
      
        % Check for self-collision at this specific point
        show(robot, current_theta, 'Collisions', 'on', 'Visuals', 'on'); drawnow;
        if checkCollision(robot, current_theta, "SkippedSelfCollisions", "parent")
            isColliding = true;
            fprintf('Self-collision detected at intermediate step %d\n', k);
            return; % Exit the function immediately
        end
    end
end