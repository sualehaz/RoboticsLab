function isCollision = checkSelfCollision1(robot,initial_angles, final_angles)

    num_samples = 50; 
    isCollision = false;
    
    for i = 1:num_samples
        fraction = i / num_samples; 
        current_config = initial_angles + fraction * (final_angles - initial_angles);
        collision_status = checkCollision(robot,current_config);

        table = collisionBox(25, 25, 1);
        table.Pose = trvec2tform([0, 0, -1]);

        if any(collision_status) || any(checkCollision(robot, current_config, {table}))
            isCollision = true;
            fprintf('WARNING: Self-collision detected at path step %d!\n', i);
            fprintf('Dangerous Angles: [%.2f, %.2f, %.2f, %.2f]\n', current_config);
            break;
        end
        show(robot, current_config, 'Collisions', 'on', 'Visuals', 'on', 'Frames','on'); 
        
        % Code for displaying an xyz axis on the end effector

        hold on; 
        
        T_ee = getTransform(robot, current_config, 'link4');
        
        pos = T_ee(1:3, 4);
        R = T_ee(1:3, 1:3);
        
        axis_len = 5; 
        
        % (X=Red, Y=Green, Z=Blue)
        quiver3(pos(1), pos(2), pos(3), R(1,1), R(2,1), R(3,1), axis_len, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
        quiver3(pos(1), pos(2), pos(3), R(1,2), R(2,2), R(3,2), axis_len, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
        quiver3(pos(1), pos(2), pos(3), R(1,3), R(2,3), R(3,3), axis_len, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);
        
        hold off;
        drawnow;
    end
    
    if ~isCollision
        disp('Path is clear! No self-collisions detected.');
    end
end
