
function setting_pose(target_x, target_y, target_z)

    a = [0 10.264 10.264 7.11];
    alpha = deg2rad([90 0 0 0]);
    d = [13.9 0 0 0];
    link_lengths = [13.9 10.264 10.264 7.11];
     
    target_phi = -pi/2; % Jaws pointing straight down (-90 degrees)
    nonColliding_th = [];
    
    
    fprintf('Testing IK for Target Pose: X=%.2f, Y=%.2f, Z=%.2f, Phi=%.2f rad\n', ...
        target_x, target_y, target_z, target_phi);
    fprintf('-------------------------------------------------------------\n');
    
    %inverse solutions
    ik_solutions = findJointAngles(target_x, target_y, target_z, target_phi)
    
    num_solutions = size(ik_solutions, 1)
    
    if num_solutions == 0
        fprintf('No valid IK solutions found. The point might be out of reach.\n');
    else
        fprintf('Found %d valid IK solutions:\n\n', num_solutions);
    
        for i = 1:num_solutions
            th = ik_solutions(i, :)
            fk_sol = getEndEffectorState2(th(1), th(2), th(3), th(4))
            fprintf('Displaying robot model: \n');
    
            % making the rigid body
            robot = rigidBodyTree("MaxNumBodies",4,"DataFormat","row");
        
            for j = 1:4
                body  = rigidBody(['link' num2str(j)]);
                joint = rigidBodyJoint(['joint' num2str(j)], 'revolute');
                setFixedTransform(joint, [a(j) alpha(j) d(j) 0], 'dh');
                body.Joint = joint;
    
                %adding in collision cylinders
                len = link_lengths(j);
                collisionObj = collisionCylinder(1.5, len); 
    
                % adusting the pose of the collision cylinders
                if j == 1
                    parentName = 'base';
                    rotation = axang2tform([1 0 0 pi/2]);
                    translation = trvec2tform([0, -len/2, 0]);
                    collisionObj.Pose = translation * rotation;
                else
                    parentName = ['link' num2str(j-1)];
                    rotation = axang2tform([0 1 0 pi/2]); 
                    translation = trvec2tform([-len/2, 0, 0]); 
                    collisionObj.Pose = translation * rotation;
                end
                
                % Add the body using the parent's NAME (string), not the joint object
                addCollision(body, collisionObj);
                addBody(robot, body, parentName);
            end
             %showdetails(robot);
            figure;
            % show(robot, th, 'Collisions', 'on', 'Visuals', 'on'); drawnow;
            title('Current Robot State');
            axis([-30 30 -30 30 0 40]);
            grid on;
                % getting current robot position
            % curr_theta = [arb.getpos(1)+deg2rad(180) arb.getpos(2)+deg2rad(90) arb.getpos(3) arb.getpos(4)]
            % out = getEndEffectorState2(curr_theta(1), curr_theta(2), curr_theta(3), curr_theta(4))
            curr_theta = [0.5 0.5 0.5 0.5]
            % check for collision
            isCollision = checkSelfCollision1(robot, curr_theta, th)
        
            if isCollision == 0
               nonColliding_th = [nonColliding_th; th];
            end
           
        end
        
        %number of solutions that don't collide
        nonColliding_th;
        %flag for checking whether the solutions fall under the range
        range_check = 1;
        %array for getting the final solution
        selected_th = [];
        if isempty(nonColliding_th)
            fprintf("All solutions are colliding\n");
        else
            %checking whether each non-colliding solution falls under the
            %required range
            for k = 1:size(nonColliding_th(1,:))
                selected_th  = nonColliding_th(k,:)
                for i = 2:4
                    if (selected_th(i) > deg2rad(150) || selected_th(i) < deg2rad(-150))
                        range_check = 0;
                    end
                end        
            end
            %via flag making sure whether a solution has been finalized
            %validating both range and collision
    
            if range_check == 1
                % arb.setpos(1,selected_th(1)-deg2rad(180),100)
                % arb.setpos(2,selected_th(2)-deg2rad(90),100)
                % arb.setpos(3,selected_th(3),100)
                % arb.setpos(4,selected_th(4),100)
    
             else
                fprintf("None of the solutions are in range\n");
             end
        end
    end
end




    
  
