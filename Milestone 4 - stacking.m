arb = Arbotix('port', 'COM3', 'nservos', 5);
arb.setpos(1, 0, 100);
arb.setpos(2, pi/2, 100);
arb.setpos(3, 0, 100);
arb.setpos(4, 0, 100);
arb.setpos(5, 0, 100);
pause(3);  

[depth_img, imgRGB, depth_data] = depth_sensor();
[redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);

% YELLOW MASK

Mask = yellowMask;
spacing = 4;

 for i = 1:3
    [targetX, targetY, depth, aligned] = current_coordinates(Mask, depth_img, imgRGB, depth_data);
    Stats = regionprops(Mask, 'BoundingBox', 'Centroid', 'Area');
    maskedDepth = double(depth_img) .* double(Mask);
    thisDepth = max(maskedDepth(:)) * 100; 
    allBlockDepths(i) = thisDepth;
    fprintf('Block %d depth: %.2f cm\n', i, thisDepth);

 % Constant: distance from camera to the work surface
    boardDistance = 45.0; 
    x_final = 0+(i-1)*spacing;
    y_final = -17;
    
    % Use MIN for the 'top' of the block, MAX for the 'base'
    maskedDepth = double(depth_img) .* double(Mask);
    pixelDepths = maskedDepth(maskedDepth > 0);
    
    if isempty(pixelDepths)
        continue;
    end
    
    % We want the point closest to the camera (the top block)
    topDepth = min(pixelDepths) * 10
    blockHeight = boardDistance - topDepth

    blockHeight = topDepth 
    blockHeight = blockHeight*2
    
    % Get coordinates for the robotic arm

    fprintf('Object %d: Height is %.2f cm at [%.1f, %.1f]\n', i, blockHeight, targetX, targetY);

    % pre pick pose
    if i == 1 
        setting_pose(arb, targetX-1, targetY-1, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX-1, targetY-1, blockHeight+3) 
    elseif i ==2
         setting_pose(arb, targetX-1, targetY-1, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX-1, targetY-1, blockHeight)
    else 
        setting_pose(arb, targetX-1, targetY-1, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX-1, targetY-1, blockHeight-3)
    end

    % grasping pose
    arb.setpos(5, 1.3, 100);
    pause(2);
    
    % slow to avoid knocking over the pile
    arb.setpos(2, 0.5, 50);
    pause(4);
    
    setting_pose(arb, x_final, y_final, 14 + ((i-2)*3))

    setting_pose(arb, x_final, y_final, 6.5)
    pause(4);

    arb.setpos(5, 0, 100);
    pause(4);

    % initial 
    arb.setpos(2, pi/2, 100);
    pause(2);
    arb.setpos(3, 0, 100);
    pause(2);
    arb.setpos(1, 0, 100);
    arb.setpos(4, 0, 100);
    arb.setpos(5, 0, 100);
    pause(3);  
        
    [depth_img, imgRGB, depth_data] = depth_sensor();
    [redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);
end



% GREEN MASK

Mask = greenMask;
spacing = 4;

 for i = 1:2
    [targetX, targetY, depth, aligned] = current_coordinates(Mask, depth_img, imgRGB, depth_data);
    Stats = regionprops(Mask, 'BoundingBox', 'Centroid', 'Area');
    maskedDepth = double(depth_img) .* double(Mask);
    thisDepth = max(maskedDepth(:)) * 100; 
    allBlockDepths(i) = thisDepth;
    fprintf('Block %d depth: %.2f cm\n', i, thisDepth);

 % Constant: distance from camera to the work surface
    boardDistance = 45.0; 
    x_final = 0+(i-1)*spacing;
    y_final = 15;
    
    % Use MIN for the 'top' of the block, MAX for the 'base'
    maskedDepth = double(depth_img) .* double(Mask);
    pixelDepths = maskedDepth(maskedDepth > 0);
    
    if isempty(pixelDepths)
        continue;
    end
    
    % We want the point closest to the camera (the top block)
    topDepth = min(pixelDepths) * 10
    blockHeight = boardDistance - topDepth

    blockHeight = topDepth 
    blockHeight = blockHeight*2
    
    % Get coordinates for the robotic arm

    fprintf('Object %d: Height is %.2f cm at [%.1f, %.1f]\n', i, blockHeight, targetX, targetY);

    % pre pick pose
    if i ==1
         setting_pose(arb, targetX, targetY, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX, targetY, blockHeight-1)
    else 
        setting_pose(arb, targetX, targetY, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX, targetY, blockHeight-3)
    end

    % grasping pose
    arb.setpos(5, 1.3, 100);
    pause(2);
    
    % slow to avoid knocking over the pile
    arb.setpos(2, 0.5, 50);
    pause(4);
    
    setting_pose(arb, x_final, y_final, 14 + ((i-2)*3))

    setting_pose(arb, x_final, y_final, 6.5)
    pause(4);

    arb.setpos(5, 0, 100);
    pause(4);

    % initial 
    arb.setpos(2, pi/2, 100);
    pause(2);
    arb.setpos(3, 0, 100);
    pause(2);
    arb.setpos(1, 0, 100);
    arb.setpos(4, 0, 100);
    arb.setpos(5, 0, 100);
    pause(3);  
        
    [depth_img, imgRGB, depth_data] = depth_sensor();
    [redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);
 end

 fprintf("All blocks placed <3 \n");
