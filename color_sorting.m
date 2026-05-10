% dedicate a spot for color 1 placement
% identify color 1
% place color 1
% dedicate a spot for color 2 placement
% identify color 2
% place color 2 

function color_sorting 

   % Starting from default position
    arb = Arbotix('port', 'COM3', 'nservos', 5);
    arb.setpos(1, 0, 100);
    arb.setpos(2, pi/2, 100);
    arb.setpos(3, 0, 100);
    arb.setpos(4, 0, 100);
    arb.setpos(5, 0, 100);
    pause(2);    

    % Initializing Masks 
    [depth_img, imgRGB, depth_data] = depth_sensor();
    imshow(depth_img)
    [redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);
    
    %select colors to be sorted
    [labeledImage1, numObjects1] = bwlabel(blueMask);
    [labeledImage2, numObjects2] = bwlabel(redMask);

    spacing = 4; 

    for i = 1:numObjects1
        objectNumber = i;
        
        objectMask = (labeledImage1 == objectNumber);
        imshow(objectMask)
        
        % Getting the coordinates of the block with the selected color
        [x_initial, y_initial, depth, aligned] = current_coordinates(objectMask);
        
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
            arb.setpos(2, 0, 100);
            pause(1)
            
            % Going to specified destination coordinates in pre-place position
            err = setting_pose(arb, x_final, y_final, 14) % keeping it higher to prevent collision
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
            pause(2)
        end
    end
    
    
        for i = 1:numObjects2
        objectNumber = i;
        
        objectMask = (labeledImage2 == objectNumber);
        imshow(objectMask)
        
        % Getting the coordinates of the block with the selected color
        [x2_initial, y2_initial, depth, aligned] = current_coordinates(objectMask);
        
        % Destination coordinates of the block
          x2_final = 0+(i-1)*spacing;
          y2_final = -15;

        % checking orientation of the block
        
        if aligned == 1
            % Going to pre-grasp position 
            err = setting_pose(arb, x2_initial, y2_initial, 14);
            if err == 1, return; end
            
            % Going to grasp position
            err = setting_pose(arb, x2_initial, y2_initial, 6.5);
            if err == 1, return; end
            
            % Grasping the block (angle5 = 1.3) 
            arb.setpos(5, 1.3, 100);

            % avoiding inter-block collision
            pause(2)
            arb.setpos(2, 0, 100);
            pause(1)
            
            % Going to specified destination coordinates in pre-place position
            err = setting_pose(arb, x2_final, y2_final, 14) % keeping it higher to prevent collision
            if err == 1, return; end
            
            % Going to place position.
            err = setting_pose(arb, x2_final, y2_final, 6.5)
            if err == 1, return; end
            
            pause(1);

            % Ungrasping the block (angle5 = 0)
            pause(1);
            arb.setpos(5, 0, 100);
            pause(1);
            
            %going back to max pre-place
            setting_pose(arb, x2_final, y2_final, 14);
            if err == 1, return; end
            
            pause(1);
            pause(2)
        end
        end
    arb.setpos(2, pi/2, 100);
    arb.setpos(1, 0, 100);
    arb.setpos(3, 0, 100);
    arb.setpos(4, 0, 100);
    

end

   
