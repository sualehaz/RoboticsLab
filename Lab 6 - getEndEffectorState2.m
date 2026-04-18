
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

    out = [x, y, z, phi_deg];
end
