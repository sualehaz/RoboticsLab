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
    pause(3);    
    % Initializing Masks 
    [depth_img, imgRGB, depth_data] = depth_sensor();
    imshow(depth_img)
    [redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);
    
    [labeledImage, numObjects] = bwlabel(yellowMask);

    spacing = 4;

    
    for i = 1:numObjects
        objectNumber = i;
        
        objectMask = (labeledImage == objectNumber);
        imshow(objectMask)
        
        % Getting the coordinates of the block with the selected color
        [x_initial, y_initial, depth, aligned] = current_coordinates(objectMask,depth_img, imgRGB, depth_data);
        
        % Destination coordinates of the block
        x_final = 0+(i-1)*spacing;
        y_final = 15;
        % checking orientation of the block
        
        if aligned == 1
            % Going to pre-grasp position 
            err = setting_pose(arb, x_initial, y_initial, 14);
            if err == 1, return; end
            
            % Going to grasp position
            err = setting_pose(arb, x_initial, y_initial, 6.5);
            if err == 1, return; end
            
            % Grasping the block (angle5 = 1.3) 
            arb.setpos(5, 1.3, 100);

            % avoiding inter-block collision
            pause(2)
            arb.setpos(2, 0.5, 100);
            pause(3)
            
            % Going to specified destination coordinates in pre-place position
            err = setting_pose(arb, x_final, y_final, 14) % keeping it higher to prevent collision
            pause(3)
            if err == 1, return; end
            
            % Going to place position.
            err = setting_pose(arb, x_final, y_final, 6.5)
            if err == 1, return; end
            
            pause(1);

            % Ungrasping the block (angle5 = 0)
            pause(1);
            arb.setpos(5, 0, 100);
            pause(1);
            
            %going back to max pre-place
            setting_pose(arb, x_final, y_final, 14);
            if err == 1, return; end
            
            pause(1);
            
            x_initial
            y_initial
             
            % Robot going back to default position 
            
            pause(2)
             
            % if abs(x_initial - x_final)<=1.5 && abs(y_initial - y_final)<1.5
            % 
            %     break;
            % end
        end
    end
    fprintf("All cubes placed\n")
    arb.setpos(2, pi/2, 100);
    arb.setpos(1, 0, 100);
    arb.setpos(3, 0, 100);
    arb.setpos(4, 0, 100);
    fprintf("All cubes placed <3")

end

