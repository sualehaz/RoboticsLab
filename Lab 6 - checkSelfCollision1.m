function isCollision = checkSelfCollision1(robot,initial_angles, final_angles)

    num_samples = 50; 
    isCollision = false;
    % 
    % % fix for random error issue (jugadh XD) 
    % if length(initial_angles) == 3
    %     initial_angles = [initial_angles(:); 0];
    % end

    for i = 1:num_samples
        fraction = i / num_samples; 
        current_config = initial_angles + fraction * (final_angles - initial_angles);
        collision_status = checkCollision(robot,current_config, 'SkippedSelfCollisions', 'adjacent');

        table = collisionBox(25, 25, 1);
        table.Pose = trvec2tform([0, 0, -1]);

        if any(collision_status) || any(checkCollision(robot, current_config, {table}))
            isCollision = true;
            fprintf('WARNING: Self-collision detected at path step %d!\n', i);
            fprintf('Dangerous Angles: [%.2f, %.2f, %.2f, %.2f]\n', current_config);
            break;
        end
    end
    
    if ~isCollision
        disp('Path is clear! No self-collisions detected.');
    end
end
