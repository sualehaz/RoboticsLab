function out = getEndEffectorState2(t1, t2, t3, t4)
    % DH Parameters
    a = [0 10.264 10.264 7.11];
    alpha = deg2rad([90 0 0 0]);
    d = [13.9 0 0 0];
    theta = [t1, t2, t3, t4]; 
    link_lengths = [13.9 10.264 10.264 7.11];
    
    % --- Kinematics Calculation ---
    T = eye(4);
    for i = 1:4
        th = theta(i); di = d(i); ai = a(i); al = alpha(i);
        Ti = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) ai*cos(th);
              sin(th)  cos(th)*cos(al) -cos(th)*sin(al) ai*sin(th);
              0        sin(al)          cos(al)         di;
              0        0                0               1];
        T = T * Ti;
    end
    x = T(1,4); y = T(2,4); z = T(3,4);
    phi_deg = rad2deg(t2 + t3 + t4);

    % --- Robot Model Construction ---
    robot = rigidBodyTree("MaxNumBodies",4,"DataFormat","row");

    for i = 1:4
        body  = rigidBody(['link' num2str(i)]);
        joint = rigidBodyJoint(['joint' num2str(i)], 'revolute');
        setFixedTransform(joint, [a(i) alpha(i) d(i) 0], 'dh');
        body.Joint = joint;
        
        len = link_lengths(i);
        % Increased radius to 1.5 for better visibility
        collisionObj = collisionCylinder(1.5, len); 
        
        if i == 1
            % Link 1 (The Base Pillar)
            % 1. Rotate the Z-pointing cylinder to align with the Y-axis of Frame 1
            % (This makes it stand vertically relative to the ground)
            rotation = axang2tform([1 0 0 pi/2]); 
    
    % 2. Shift it DOWN along the Y-axis by half its length to reach the base
    translation = trvec2tform([0, -len/2, 0]);
    
    collisionObj.Pose = translation * rotation;
    parentName = 'base';
              
        else
            % Rotate to X-axis and shift BACK to previous joint
            rotation = axang2tform([0 1 0 pi/2]); 
            translation = trvec2tform([-len/2, 0, 0]); 
            collisionObj.Pose = translation * rotation;
            parentName = ['link' num2str(i-1)];
        end
        
        % FIX: Add collision to the body BEFORE adding the body to the robot
        addCollision(body, collisionObj);
        addBody(robot, body, parentName);
    end

    % --- Visualization ---
    figure;
    % FIX: Set 'Visuals' to 'on' but keep 'Collisions' on. 
    % This ensures the collision meshes are rendered.
    show(robot, theta, 'Collisions', 'on', 'Visuals', 'on'); 
    title('Phantom X Pincher: Collision Model');
    axis([-30 30 -30 30 0 40]);
    grid on; view(3); % Ensures 3D view

    collision = checkSelfCollision(robot, theta, [0.2861   -0.6211  1.0409   -1.9906]);
    out = [x, y, z, phi_deg, collision];
end