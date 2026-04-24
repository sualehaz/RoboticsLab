% PICK AND PLACE MOTION PIPELINE
% arb = Arbotix('port', 'COM3', 'nservos', 5);

% Starting from default position
arb.setpos(1, 0, 100);
arb.setpos(2, 0, 100);
arb.setpos(3, 0, 100);
arb.setpos(4, 0, 100);
arb.setpos(5, 0, 100);

x=12.7;
y=-14.2;

x_final = 11;
y_final = 14;

% Going to pre-grasp position (z = 8) CASE 1
setting_pose(arb, x, y, 8)

% Going to grasp position (z=6) CASE 1
setting_pose(arb, x, y, 6)

% Going to pre-grasp position (z = 10) CASE 2
 %setting_pose(arb, x, y, 10)

% Going to grasp position (z=6) CASE 2
 %setting_pose(arb, x, y, 6)

% Grasping the block (angle5 = 1.3) 
arb.setpos(5, 1.3, 100);

% Going to specified destination coordinates in pre-place position (z = 8)
% CASE 1
setting_pose(arb, x_final, y_final, 8)

% Going to place position (z=4) CASE 1
setting_pose(arb, x_final, y_final, 6)

pause(1);
% Going to specified destination coordinates in pre-place position (z = 10)
% CASE 2
 %setting_pose(arb, x_final, y_final, 10)

% Going to place position (z=6) CASE 2
% setting_pose(arb, x_final, y_final, 6)
%pause(1);
% Ungrasping the block (angle5 = 0)
 arb.setpos(5, 0, 100);
 pause(1);

 %going back to pre-place
 setting_pose(arb, x_final, y_final, 8);
 pause(1);

 
% Robot going back to default position 
arb.setpos(2, 0, 100);
arb.setpos(1, 0, 100);
arb.setpos(3, 0, 100);
arb.setpos(4, 0, 100);



