function isCollision = checkSelfCollision1(robot,initial_angles, final_angles)

    num_samples = 50; 
    isCollision = false;
    
    for i = 1:num_samples
        fraction = i / num_samples; 
        current_config = initial_angles + fraction * (final_angles - initial_angles);
        collision_status = checkCollision(robot,current_config,"SkippedSelfCollisions", "adjacent");
        
        if any(collision_status)
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
