arb = Arbotix('port', 'COM3', 'nservos', 5);
arb.setpos(1, 0, 100);
arb.setpos(2, pi/2, 100);
arb.setpos(3, 0, 100);
arb.setpos(4, 0, 100);
arb.setpos(5, 0, 100);
pause(3);  

[depth_img, imgRGB, depth_data] = depth_sensor();
[redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);

Mask = yellowMask;
spacing = 4;
% 
% [labeledImage, numObjects] = bwlabel(Mask);
% allBlockDepths = zeros(1, numObjects);

 for i = 1:3
    % objectMask = (labeledImage == i);
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
    
    % objectMask = (labeledImage == i);
    
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
        setting_pose(arb, targetX, targetY, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX, targetY, blockHeight+3) 
    elseif i ==2
         setting_pose(arb, targetX, targetY, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX, targetY, blockHeight-1)
    else 
        setting_pose(arb, targetX, targetY, blockHeight+6)
        pause(3);
    
        setting_pose(arb, targetX, targetY, blockHeight-3)
    end

    % setting_pose(arb, targetX, targetY, blockHeight+7)
    % pause(4);
    % fprintf('Action: Removing top block from stack...\n');

    % arb.setpos(5, 1.3, 100);
    % pause(4);

    % picking pose
    % if i == 3
    % 	setting_pose(arb, targetX, targetY, blockHeight+2.5)
    % else
    % 	setting_pose(arb, targetX, targetY, blockHeight+3.5)
    % end
    % pause(3);

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
        
        
        % %% Execution Logic
        % if blockHeight > 5.5
        %     % CASE: STACK DETECTED
        %     fprintf('Action: Removing top block from stack...\n');
        %     % robot_pick(targetX, targetY, topDepth);
        %     % robot_place('Stack_Zone');
        % elseif blockHeight >= 2.0 && blockHeight <= 5.5
        %     % CASE: SINGLE BLOCK
        %     fprintf('Action: Picking single block...\n');
        %     % robot_pick(targetX, targetY, topDepth);
        %     % robot_place('Target_Zone');
        % else
        %     fprintf('Action: Height out of range, check for noise.\n');
        %end


%{
height from camera to board = 45 (approx)
block depth = 45 - block height

case 1: 2 red square blocks + 1 yellow rectangle block
- block depth = 45 - 15 = 30
- test 1: 38.6584  ( +x, y = 0)
- test 2: 39.1208  ( x = 0, -y)
- test 3: 37.4335  ( x = 0, +y)
- test 4: 39.2333  ( -x, y = 0) (stacked them differently) 

block heights:
square: 3.3 cm
rectangle: 5 cm
rectangle: 2.6 cm 

%}

%{
Algorithm for stacking 

if block depth < 3.3 or < 5 or < 2.6
    recognize color
    pick and place them in designated spot 

else
   if height > 5 
      apply mask
      recognize color 
      place in position
      check height again
%}


 %{
case i: square + rectangle 
- 45-8  = 37

case ii: square + square
- 45-5.6 = 39.4

 %}
    [depth_img, imgRGB, depth_data] = depth_sensor();
    [redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);

end
