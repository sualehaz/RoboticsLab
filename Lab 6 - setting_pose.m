
function err = setting_pose(arb, target_x, target_y, target_z)

    err = 0;
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
    ik_solutions = findJointAngles(target_x, target_y, target_z, target_phi);
    
    num_solutions = size(ik_solutions, 1);
    
    if num_solutions == 0
        fprintf('No valid IK solutions found. The point might be out of reach.\n');
        err = 1;
    else
        fprintf('Found %d valid IK solutions:\n\n', num_solutions);
        figure;
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

            title('Current Robot State');
            axis([-30 30 -30 30 0 40]);
            grid on;
                % getting current robot position
            curr_theta = [arb.getpos(1) arb.getpos(2)+deg2rad(90) arb.getpos(3) arb.getpos(4)]
            % out = getEndEffectorState2(curr_theta(1), curr_theta(2), curr_theta(3), curr_theta(4))
            % curr_theta = [-1.5 0.5 0.5 0.5]
            % check for collision
            isCollision = checkSelfCollision1(robot, curr_theta, th)
        
            if isCollision == 0
               nonColliding_th = [nonColliding_th; th];
            end
           
        end
        figure;
        %number of solutions that don't collide
        ik_solutions
        nonColliding_th
        %flag for checking whether the solutions fall under the range
        range_check = 1;
        %array for getting the final solution
        selected_th = [];
        %array of solutions that are clear to execute
        acceptable_sol = [];
        if isempty(nonColliding_th)
            fprintf("All solutions are colliding\n");
            err = 1;
        else
            %checking whether each non-colliding solution falls under the
            %required range
            for k = 1:length(nonColliding_th(:, 1))
                selected_th  = nonColliding_th(k,:);
                range_check = 1;
                for i = 1:4
                    if (selected_th(i) > deg2rad(149) || selected_th(i) < deg2rad(-150))
                        range_check = 0;
                    end
                end 
                % Adding into the array of acceptable solutions 
                if range_check == 1
                    acceptable_sol = [acceptable_sol; selected_th]
                end       
            end

            if isempty(acceptable_sol)
                fprintf('All solutions are out of range\n');
                err = 1;
            else

                % finding the optimal solution
                optimal = find_optimal(curr_theta,acceptable_sol)
        
                if isempty(optimal)
                    fprintf("None of the solutions are in range\n");
                    
                else
       
                    arb.setpos(1,optimal(1),100)
                    arb.setpos(2,optimal(2)-deg2rad(90),100)
                    arb.setpos(3,optimal(3),100)
                    arb.setpos(4,optimal(4),100)
                end
            end
        end
    end
end




    
  
