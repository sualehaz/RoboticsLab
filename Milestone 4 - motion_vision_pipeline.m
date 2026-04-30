% PICK AND PLACE MOTION PIPELINE
% arb = Arbotix('port', 'COM3', 'nservos', 5);
function motion_vision_pipeline

    % Starting from default position
    arb = Arbotix('port', 'COM3', 'nservos', 5);
    arb.setpos(1, 0, 100);
    arb.setpos(2, pi/2, 100);
    arb.setpos(3, 0, 100);
    arb.setpos(4, 0, 100);
    arb.setpos(5, 0, 100);
    
    % Initializing Masks 
    [redMask, blueMask, greenMask, yellowMask] = creating_masks();
    
    % Getting the coordinates of the block with the selected color
    [x_initial, y_initial, depth] = current_coordinates(redMask);
    
    % Destination coordinates of the block
    x_final = 0;
    y_final = 15;
    
    % -------------------- CASE 1 ---------------------------------------------
    while (nnz(redMask) > 10)
        if depth < 47
        
            % Going to pre-grasp position (z = 8)
            err = setting_pose(arb, x_initial, y_initial, 12);
            if err == 1, return; end
            
            % Going to grasp position (z=6) CASE 1
            err = setting_pose(arb, x_initial, y_initial, 7);
            if err == 1, return; end
        
        end
        
        % -------------------- CASE 2 ---------------------------------------------
        
        if depth >= 47
        
            % Going to pre-grasp position (z = 8)
            err = setting_pose(arb, x_initial, y_initial, 12)
            if err == 1, return; end
            
            % Going to grasp position (z=6) CASE 1
            err = setting_pose(arb, x_initial, y_initial, 7)
            if err == 1, return; end
        
        end
        
        % Grasping the block (angle5 = 1.3) 
        arb.setpos(5, 1.3, 100);
        
        % Going to specified destination coordinates in pre-place position (z = 8)
        % -------------------- CASE 1 ---------------------------------------------
        if depth < 47
            err = setting_pose(arb, x_final, y_final, 12) % keeping it higher to prevent collision
            if err == 1, return; end
            
            % Going to place position.
            err = setting_pose(arb, x_final, y_final, 7)
            if err == 1, return; end
        end
        
        pause(1);
        
        
        % Going to specified destination coordinates in pre-place position (z = 10)
        % -------------------- CASE 2 ---------------------------------------------
        if depth >= 47
        
            err = setting_pose(arb, x_final, y_final, 12) % keeping it higher to prevent collision
            if err == 1, return; end
            
            % Going to place position
            err = setting_pose(arb, x_final, y_final, 7)
            if err == 1, return; end
        
        end
        
        
        % Ungrasping the block (angle5 = 0)
        pause(1);
        arb.setpos(5, 0, 100);
        pause(1);
        
        %going back to max pre-place
        setting_pose(arb, x_final, y_final, 12);
        if err == 1, return; end
        
        pause(1);
        
        x_initial
        y_initial
         
        % Robot going back to default position 
        arb.setpos(2, pi/2, 100);
        arb.setpos(1, 0, 100);
        arb.setpos(3, 0, 100);
        arb.setpos(4, 0, 100);
        pause(2)
        [redMask, blueMask, greenMask, yellowMask] = creating_masks();
        [x_initial, y_initial, depth] = current_coordinates(redMask);
        if abs(x_initial - x_final)<=1.5 && abs(y_initial - y_final)<1.5
            fprintf("All cubes placed\n");
            break;
        end
    end

end

