function varargout = RenderSPM(theta, axes, home_core_ends, rot_render, record_data, path, data, last_yaw, optns)
    arguments
        theta (3,1) {mustBeNumeric}
        axes
        home_core_ends (3,3) {mustBeNumeric}
        rot_render (3,3)
        record_data double
        path
        data
        last_yaw double

        optns.home_pos_override (3,3) = NaN
        optns.validate double = 0
    end
    base_height = 5;

    upper_segment_centre_dist = 27.5;
    lower_segment_centre_dist = 32.5;
    
    % Angle constants
    a_1 = deg2rad(45);
    a_2 = deg2rad(40);
    beta = deg2rad(62.435);
    % a_2 = deg2rad(90);
    % beta = deg2rad(90);
    a_3 = 2 * asin(sin(beta) * cos(pi/6));

    % Calculate joint axes
    w=zeros(3);
    for i=1:3 % Determine w axes
        n_i = 2 * (i-1) * pi / 3;
        w(:,i) = [sin(n_i-theta(i)) * sin(a_1);
                  cos(n_i-theta(i)) * sin(a_1);
                  -cos(a_1)];
    end
    % Numerically calculate v axes
    eqs = @(v) [Dot(w(:,1),v(:,1)) - cos(a_2)
               Dot(w(:,2),v(:,2)) - cos(a_2)
               Dot(w(:,3),v(:,3)) - cos(a_2)
        
               Dot(v(:,1),v(:,2)) - cos(a_3)
               Dot(v(:,1),v(:,3)) - cos(a_3)
               Dot(v(:,2),v(:,3)) - cos(a_3)
        
               norm(v(:,1)) - 1
               norm(v(:,2)) - 1
               norm(v(:,3)) - 1];

    % Using signs of the home position to get faster, and consistently oriented solutions
    home_pos = [1,1,-1;1,-1,1;-1,-1,-1];
    if ~isnan(optns.home_pos_override)
        home_pos = optns.home_pos_override;
    end
    v = double(fsolve(eqs,home_pos,optimset('Display','off')));
    if numel(v) ~= 9
        disp("Failed to solve")
        return
    end

    % Draw Base axis
    base_axis_origin = rot_render * [0,0,-lower_segment_centre_dist]';
    base_axis_vec = rot_render * [0,0,2*base_height]';
    quiver3vec(base_axis_origin,base_axis_vec,"b",1.5,'on',1.5)

    % Draw lower arms
    arm_points = w * lower_segment_centre_dist;
    TriplePlot(TripleRotate(arm_points, rot_render),rot_render*[0;0;-lower_segment_centre_dist]);

    % Draw lower stage axes
    quiver3vec(TripleRotate(arm_points, rot_render),TripleRotate(-5*w,rot_render),"g",1.5,'on',1.5)

    % Draw upper arms
    upper_segment_ends = v * upper_segment_centre_dist;
    upper_segment_ends_render = TripleRotate(upper_segment_ends, rot_render);
    TriplePlot(TripleRotate(arm_points-5*w, rot_render),upper_segment_ends_render);
    if optns.validate == 1
        upper_segment_start_render = TripleRotate(arm_points-5*w, rot_render);
        l1 = norm(upper_segment_ends_render(:,1)-upper_segment_start_render(:,1));
        l2 = norm(upper_segment_ends_render(:,2)-upper_segment_start_render(:,2));
        l3 = norm(upper_segment_ends_render(:,3)-upper_segment_start_render(:,3));
        if abs(l1 - l2) > 0.1 || abs(l2 - l3) > 0.1 || abs(l1 - l3) > 0.1
            varargout = {0};
            return
        end
    end
    % Draw upper stage axes
    quiver3vec(upper_segment_ends_render,TripleRotate(-5*v,rot_render),"r",1.5,'on',1.5);

    % Draw core (platform)
    core_ends = upper_segment_ends - 5*v;
    midpoint = double(core_ends(:,1) + core_ends(:,2) + core_ends(:,3)) ./ 3;
    midpoint_render = rot_render * midpoint;
    plot3(midpoint_render(1),midpoint_render(2),midpoint_render(3),"r o");
    TriplePlot(rot_render * core_ends,midpoint_render);

    % Get current orientation normal
    delta_1 = core_ends(:,1) - midpoint;
    delta_2 = core_ends(:,2) - midpoint;
    normal = Cross(delta_2,delta_1); normal = normal / norm(normal);

    % Determine the angle between the home position normal and new normal, 
    % as well as the axis of that rotation
    midpoint_orig = double(home_core_ends(:,1) + home_core_ends(:,2) + home_core_ends(:,3)) ./ 3;
    delta_orig_1 = home_core_ends(:,1) - midpoint_orig;
    delta_orig_1 = delta_orig_1 / norm(delta_orig_1);

    pitch = acos(Dot(axes(:,3),normal));
    rot_axis = Cross(axes(:,3),normal); rot_axis = rot_axis / norm(rot_axis);
    rotated = RodriguesRotate(delta_orig_1, rot_axis, pitch); rotated = rotated/norm(rotated);

    % Calculate angles and correct for flipped axes
    roll = -acos(Dot(delta_1/norm(delta_1),rotated));

    if abs(pitch) > 1e-3
        if Dot(axes(:,2),rot_axis) < 0
            rot_axis = -rot_axis;
            roll = -roll; pitch = -pitch;
        end
        yaw = acos(Dot(axes(:,1),rot_axis));
        if abs(yaw-last_yaw) > pi - 0.2
            if yaw>last_yaw
                yaw = yaw-pi;
            else
                yaw = yaw+pi;
                roll = -roll; pitch = -pitch;
            end

        end
    else
        yaw = roll;
        roll = 0; pitch = 0;
        if abs(yaw-last_yaw) > pi/2
            yaw = last_yaw;
        end
        cross = Cross(delta_1/norm(delta_1),rotated);
        if cross(3) < 0
            yaw = -yaw;
        end
    end
    % roll = -atan2(dot(Cross(delta_1,rotated),rot_axis),dot(delta_1,rotated));
    % yaw = -atan2(dot(Cross(axes(:,1),rot_axis),axes(:,2)),dot(axes(:,1),rot_axis));

    euler = [yaw,pitch,roll];
    rot = rot_render * Euler2Transform(euler);
    
    % Data output
    if record_data == 1 
        data = [data;euler];
        path = [path,midpoint];
        plot3(path(1,:),path(2,:),path(3,:),"g-")
    end

    if optns.validate == 0
        if record_data == 1
            varargout = {path,data,rot};
        else
            varargout = {rot};
        end
    else
        varargout = {1};
    end
end

%% Helper funcs
function a = Dot(u,v)
    a = sum(u .* v);
end
function a = Cross(u,v)
    a = [u(2)*v(3) - u(3)*v(2)
         -u(1)*v(3) + u(3)*v(1)
         u(1)*v(2) - u(2)*v(1)];
end

% Rotate 3D 'vector' centred on the origin, about a normalised 'axis' 
% vector by 'angle' radians
function out = RodriguesRotate(vector, axis, angle)
    out = (vector * cos(angle)) + ...
        (Cross(axis,vector) * sin(angle)) + ...1    
        (axis * Dot(axis, vector) * (1-cos(angle)));
end

function out = TripleRotate(vectors,rot)
    out(:,1) = rot * vectors(:,1);
    out(:,2) = rot * vectors(:,2);
    out(:,3) = rot * vectors(:,3);
end

% plot a set of 3 (column) vectors, defined either as point to point, or
% point to common point
function TriplePlot(p1,p2)
    for i = 1:3
        if numel(p2) == 3
            plot3([p1(1,i),p2(1)],[p1(2,i),p2(2)],[p1(3,i),p2(3)],"k");
        else
            plot3([p1(1,i),p2(1,i)],[p1(2,i),p2(2,i)],[p1(3,i),p2(3,i)],"k");
        end
    end
end
function quiver3vec(origins,vecs,colour,lineWidth,showArrowHead,maxHeadSize)
    for i=1:size(origins,2)
        quiver3(origins(1,i),origins(2,i),origins(3,i),vecs(1,i),vecs(2,i),vecs(3,i), ...
            0,colour,'LineWidth',lineWidth,'ShowArrowHead',showArrowHead,'MaxHeadSize',maxHeadSize)
    end
end

function end_point = DrawLowerArm(id, theta, segment_1_x, segment_1_z, z_offset,base_height, rot_render)
    theta  = theta - (2 * id * pi / 3);
    points = [0,                       0,                       0;
              0,                       segment_1_x,             segment_1_x;
              -z_offset+id*base_height,-z_offset+id*base_height,-z_offset+segment_1_z];
    rot_mat = [cos(theta),-sin(theta),0;
               sin(theta),cos(theta),0;
               0,0,1];
    points = rot_mat * points;
    disp_points = rot_render * points;
    plot3(disp_points(1,:),disp_points(2,:),disp_points(3,:),"k-");
    end_point = points(:,3);
end