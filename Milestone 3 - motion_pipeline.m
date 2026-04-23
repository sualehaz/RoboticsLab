% PICK AND PLACE MOTION PIPELINE
arb = Arbotix('port', 'COM3', 'nservos', 5);

% Starting from default position
arb.setpos(1, 0, 100);
arb.setpos(2, 0, 100);
arb.setpos(3, 0, 100);
arb.setpos(4, 0, 100);

% Going to pre-grasp position (z = 8) CASE 1
setting_pose(arb, -12.7, 14.2, 8)

% Going to grasp position (z=4) CASE 1
setting_pose(arb, -12.7, 14.2, 4)

% Going to pre-grasp position (z = 10) CASE 2
% setting_pose(arb, target_x, target_y, 10)

% Going to grasp position (z=6) CASE 2
% setting_pose(arb, target_x, target_y, 6)

% Grasping the block (angle5 = pi/4) 
arb.setpos(5, pi/4, 100);

% Going to specified destination coordinates in pre-place position (z = 8)
% CASE 1
setting_pose(arb, 11.7, 11.3, 8)

% Going to place position (z=4) CASE 1
setting_pose(arb, 11.7, 11.3, 4)

% Going to specified destination coordinates in pre-place position (z = 10)
% CASE 2
% setting_pose(arb, target_x, target_y, 10)

% Going to place position (z=6) CASE 2
% setting_pose(arb, target_x, target_y, 6)

% Ungrasping the block (angle5 = 0)
arb.setpos(5, 0, 100);

% Robot going back to default position 
arb.setpos(1, 0, 100);
arb.setpos(2, 0, 100);
arb.setpos(3, 0, 100);
arb.setpos(4, 0, 100);
