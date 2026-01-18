function mid = Midpoint(p1,p2)
    mid = [(p1(1)+p2(1)),(p1(2)+p2(2)),(p1(3)+p2(3))] / 2;
end

function rot = RotationY(theta)
    rot = [cos(theta) 0 -sin(theta)
           0          1  0
           sin(theta) 0 cos(theta)];
end

%% Crossed 4-bar calculation and rendering
function [out_vec,norm_vec,out_rot] = DrawCross4Bar(alpha, lengths, rot, offset, debugInfo)
    % Calculate the angle theta, based on the given angles and lengths
    k = (lengths.B^2 - lengths.A^2 - lengths.C^2 - lengths.D^2) + 2*lengths.A*lengths.C*cos(alpha);
    a = -lengths.A + lengths.C*cos(alpha);
    b = -lengths.C*sin(alpha);
    
    theta = acos(k ./ (2*lengths.D*sqrt( (a.^2) + (b.^2) ))) + atan2(b,a);

    % Determine the end points of the linkages
    p = [0,  lengths.C*cos(alpha),  lengths.A-lengths.D*cos(theta), lengths.A
         0, -lengths.C*sin(alpha), -lengths.D*sin(theta),           0
         0,  0,                     0,                              0];

    % Rotate the points according to the provided matrix, and plot
    p(:,1) = rot*(p(:,1)+offset);
    p(:,2) = rot*(p(:,2)+offset);
    p(:,3) = rot*(p(:,3)+offset);
    p(:,4) = rot*(p(:,4)+offset);
    plot3(p(1,:), p(2,:), p(3,:),"k-o",MarkerSize=3,MarkerFaceColor="r");

    b_vec = p(:,3)-p(:,2);
    local_up = rot * [0,0,-1]';

    norm_vec = cross(b_vec,local_up); norm_vec = norm_vec / norm(norm_vec);
    out_vec = Midpoint(p(:,3),p(:,2));
    if debugInfo == 1
        quiver3(out_vec(1),out_vec(2),out_vec(3),norm_vec(1),norm_vec(2),norm_vec(3), ...
            "r",'LineWidth',1.5,'ShowArrowHead','on','MaxHeadSize',1.5)
    end
    
    c = cross([0,0,-1]',norm_vec);
    if c(2) < 0
        out_rot = RotationY(acos(dot(norm_vec,[0,0,-1]')));
    else
        out_rot = RotationY(-acos(dot(norm_vec,[0,0,-1]')));
    end
    % out_rot = atan2(dot(Cross([0,0,1'],norm_vec),[0,1,0]'),dot(norm_vec,[0,0,1]'));
end

function [out_vec,norm_vec] = DrawAnkleCrankLinkage(alpha, linkage_angle_offset, lengths, gear_data, rot, offset, debugInfo)
    ratio_worm_spur = -1/gear_data.spur_n;

    alpha_spur = alpha.*ratio_worm_spur;
    alpha_out = alpha_spur + linkage_angle_offset;

    % Plot worm gear
    n_rots = gear_data.worm_length / (pi*gear_data.worm_pitch);
    theta_worm_plot = linspace(0,2*pi*n_rots,gear_data.worm_n_plot);

    worm_plot = rot*[linspace(0,gear_data.worm_length,gear_data.worm_n_plot)-gear_data.worm_length/2
        gear_data.worm_radius*cos(theta_worm_plot)
        gear_data.worm_radius*sin(theta_worm_plot)];

    worm = plot3(worm_plot(1,:) + offset(1), ...
         worm_plot(2,:) + offset(2), ...
         worm_plot(3,:) + offset(3),"r.",'MarkerSize',0.1);
    rotate(worm,rot*gear_data.worm_axis,rad2deg(alpha),offset);

    % Plot spur gear
    spur_centre = offset + rot*gear_data.spur_centre';
    spur_plot = linspace(0,2*pi,50);
    plot3(gear_data.spur_radius*cos(spur_plot)+spur_centre(1)*ones(size(spur_plot)), ...
        spur_centre(2)*ones(size(spur_plot)), ...
        gear_data.spur_radius*sin(spur_plot)+spur_centre(3)*ones(size(spur_plot)), ...
        "b-")

    k = (lengths.C^2 - lengths.A^2 - lengths.B^2 - lengths.D^2) + 2*lengths.A*lengths.B*cos(alpha_out);
    a = lengths.A - lengths.B*cos(alpha_out);
    b = lengths.B*sin(alpha_out);
    
    theta = asin(k ./ (2*lengths.D*sqrt( (a.^2) + (b.^2) ))) + atan2(b,a);

    % Determine the end points of the linkages
    p = [-lengths.B*sin(alpha_out), -lengths.D*cos(theta),           0
          0,                     0,                              0
         -lengths.B*cos(alpha_out), -lengths.A-lengths.D*sin(theta),-lengths.A];

    % Rotate the points according to the provided matrix, and plot
    p(:,1) = rot*p(:,1) + spur_centre;
    p(:,2) = rot*p(:,2) + spur_centre;
    p(:,3) = rot*p(:,3) + spur_centre;
    plot3(p(1,:), p(2,:), p(3,:),"k-o",MarkerSize=3,MarkerFaceColor="r");

    b_vec = p(:,3)-p(:,2);
    local_up = rot * [0,-1,0]';

    norm_vec = 10*cross(b_vec/norm(b_vec),local_up);
    out_vec = Midpoint(p(:,3),p(:,2));
    if debugInfo == 1
        quiver3(out_vec(1),out_vec(2),out_vec(3),norm_vec(1),norm_vec(2),norm_vec(3),...
            "r",'LineWidth',1.5,'ShowArrowHead','on','MaxHeadSize',1.5)
    end
end

%% Run
clc; clear; close all

%% Knee 4-bar constants
A = 20;
B = 10;
C = 25;
D = 30;
knee_lengths = struct('A',A,'B',B,'C',C,'D',D);
knee_linkage_offset = [-A/2,-40,0]';

%% Ankle 4-bar constants
ankle_offset = 40;
ankle_lengths = struct('A',10,'B',3,'C',18,'D',10);

ankle_linkage_angle_offset = deg2rad(140);

gear_data = struct('worm_pitch',0.4,'worm_length',8,'worm_radius',2,'worm_n_plot',200,'worm_axis',[1,0,0]', ...
    'spur_pitch',0.4,'spur_n',24,'spur_axis',[0,1,0]');
gear_data.spur_radius = gear_data.spur_pitch*gear_data.spur_n / 2;
gear_data.spur_centre = [0,0,-gear_data.spur_radius - gear_data.worm_radius];

rotation_amount = 12*pi;

%% SPM constants
home_core_ends = [14.2430    4.9711  -19.2142
                  13.9634  -19.3165    5.3531
                 -10.4120  -10.4120  -10.4120];
%       x y z
root_axes = [1 0 0
             0 1 0
             0 0 1];
rot_spm_render = [1  0  0
                  0  0  -1
                  0 1  0];

%%
write_video = 0;

if write_video == 1
    frame_rate = 30;  % Frame rate is the inverse of the time step
    videoFile = 'Sim_Leg.mp4';  % Define the video file name
    v = VideoWriter(videoFile, 'MPEG-4');  % Create VideoWriter object for MP4 file
    v.FrameRate = frame_rate;  % Set the frame rate of the video
    % Open the video file
    open(v);
end

%% Render

last_yaw = NaN;

f = figure(WindowState="maximized");
ax1 = subplot(1,2,1);
ax2 = subplot(1,2,2);

pause(1);

path = []; data = [];

%%
subplot(ax1);
plot3(0,0,0,"x b");

hold on
[path,data,rot] = RenderSPM([deg2rad(-5),deg2rad(-5),deg2rad(-5)],root_axes,home_core_ends,rot_spm_render,1,path,data,last_yaw);
last_yaw = rot(1);
[knee_out,knee_norm,knee_out_rot] = DrawCross4Bar(deg2rad(55+30),knee_lengths,RotationY(pi/4)*rot,knee_linkage_offset,0);
ankle_draw_offset = knee_out' + (ankle_offset*RotationY(deg2rad(-40))*(knee_norm/norm(knee_norm)));
DrawAnkleCrankLinkage(deg2rad(1200),ankle_linkage_angle_offset,ankle_lengths,gear_data,RotationY(deg2rad(-40))*knee_out_rot,ankle_draw_offset,1);

drawn_knee_linkage_offset = RotationY(pi/4)*rot*knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
drawn_knee_linkage_offset_p1 = drawn_knee_linkage_offset - RotationY(pi/4)*rot*[A/2,0,0]';
drawn_knee_linkage_offset_p2 = drawn_knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
plot3([0,drawn_knee_linkage_offset(1),drawn_knee_linkage_offset_p1(1),drawn_knee_linkage_offset_p2(1)], ...
    [0,drawn_knee_linkage_offset(2),drawn_knee_linkage_offset_p1(2),drawn_knee_linkage_offset_p2(2)], ...
    [0,drawn_knee_linkage_offset(3),drawn_knee_linkage_offset_p1(3),drawn_knee_linkage_offset_p2(3)], ...
    "k-",LineWidth=1.5);
plot3([knee_out(1),ankle_draw_offset(1)], ...
    [knee_out(2),ankle_draw_offset(2)], ...
    [knee_out(3),ankle_draw_offset(3)], ...
    "k-",LineWidth=1.5)

axis equal
axis([-30,140,-20,40,-140,40]);
hold off

CopyToOrthAxes(ax1, ax2);
%%

for i = deg2rad(-30:0.5:30)
    % continue
    subplot(ax1);
    plot3(0,0,0,"x b");

    hold on
    [path,data,rot] = RenderSPM([i,i,i],root_axes,home_core_ends,rot_spm_render,1,path,data,last_yaw);
    last_yaw = rot(1);
    [knee_out,knee_norm,knee_out_rot] = DrawCross4Bar(deg2rad(55),knee_lengths,RotationY(pi/4)*rot,knee_linkage_offset,0);
    ankle_draw_offset = knee_out' + (ankle_offset*RotationY(deg2rad(-40))*(knee_norm/norm(knee_norm)));
    DrawAnkleCrankLinkage(deg2rad(0),ankle_linkage_angle_offset,ankle_lengths,gear_data,RotationY(deg2rad(-40))*knee_out_rot,ankle_draw_offset,1);

    drawn_knee_linkage_offset = RotationY(pi/4)*rot*knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
    drawn_knee_linkage_offset_p1 = drawn_knee_linkage_offset - RotationY(pi/4)*rot*[A/2,0,0]';
    drawn_knee_linkage_offset_p2 = drawn_knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
    plot3([0,drawn_knee_linkage_offset(1),drawn_knee_linkage_offset_p1(1),drawn_knee_linkage_offset_p2(1)], ...
        [0,drawn_knee_linkage_offset(2),drawn_knee_linkage_offset_p1(2),drawn_knee_linkage_offset_p2(2)], ...
        [0,drawn_knee_linkage_offset(3),drawn_knee_linkage_offset_p1(3),drawn_knee_linkage_offset_p2(3)], ...
        "k-",LineWidth=1.5);
    plot3([knee_out(1),ankle_draw_offset(1)], ...
        [knee_out(2),ankle_draw_offset(2)], ...
        [knee_out(3),ankle_draw_offset(3)], ...
        "k-",LineWidth=1.5)

    axis equal
    axis([-30,140,-20,40,-140,40]);
    hold off

    CopyToOrthAxes(ax1, ax2);


    if write_video == 1
        drawnow
        frame = getframe(gcf);  % Get current frame
        writeVideo(v, frame);   % Write the frame to the video
    end
    pause(0.1)
end

%%
for j = deg2rad(55:0.5:105)
    subplot(ax1);
    plot3(0,0,0,"x b");

    hold on
    rot = RenderSPM([i,i,i],root_axes,home_core_ends,rot_spm_render,0,[],[],last_yaw);
    last_yaw = rot(1);
    [knee_out,knee_norm,knee_out_rot] = DrawCross4Bar(j,knee_lengths,RotationY(pi/4)*rot,knee_linkage_offset,0);
    ankle_draw_offset = knee_out' + (ankle_offset*RotationY(deg2rad(-40))*(knee_norm/norm(knee_norm)));
    DrawAnkleCrankLinkage(deg2rad(0),ankle_linkage_angle_offset,ankle_lengths,gear_data,RotationY(deg2rad(-40))*knee_out_rot,ankle_draw_offset,1);

    drawn_knee_linkage_offset = RotationY(pi/4)*rot*knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
    drawn_knee_linkage_offset_p1 = drawn_knee_linkage_offset - RotationY(pi/4)*rot*[A/2,0,0]';
    drawn_knee_linkage_offset_p2 = drawn_knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
    plot3([0,drawn_knee_linkage_offset(1),drawn_knee_linkage_offset_p1(1),drawn_knee_linkage_offset_p2(1)], ...
        [0,drawn_knee_linkage_offset(2),drawn_knee_linkage_offset_p1(2),drawn_knee_linkage_offset_p2(2)], ...
        [0,drawn_knee_linkage_offset(3),drawn_knee_linkage_offset_p1(3),drawn_knee_linkage_offset_p2(3)], ...
        "k-",LineWidth=1.5);
    plot3([knee_out(1),ankle_draw_offset(1)], ...
        [knee_out(2),ankle_draw_offset(2)], ...
        [knee_out(3),ankle_draw_offset(3)], ...
        "k-",LineWidth=1.5)

    axis equal
    axis([-30,140,-20,40,-140,40]);
    hold off

    CopyToOrthAxes(ax1, ax2);


    if write_video == 1
        drawnow
        frame = getframe(gcf);  % Get current frame
        writeVideo(v, frame);   % Write the frame to the video
    end
    pause(0.1)
end
%%
for k = linspace(0,rotation_amount,50)
    subplot(ax1);
    plot3(0,0,0,"x b");

    hold on
    rot = RenderSPM([i,i,i],root_axes,home_core_ends,rot_spm_render,0,[],[],last_yaw);
    last_yaw = rot(1);
    [knee_out,knee_norm,knee_out_rot] = DrawCross4Bar(j,knee_lengths,RotationY(pi/4)*rot,knee_linkage_offset,0);
    ankle_draw_offset = knee_out' + (ankle_offset*RotationY(deg2rad(-40))*(knee_norm/norm(knee_norm)));
    DrawAnkleCrankLinkage(k,ankle_linkage_angle_offset,ankle_lengths,gear_data,RotationY(deg2rad(-40))*knee_out_rot,ankle_draw_offset,1);

    drawn_knee_linkage_offset = RotationY(pi/4)*rot*knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
    drawn_knee_linkage_offset_p1 = drawn_knee_linkage_offset - RotationY(pi/4)*rot*[A/2,0,0]';
    drawn_knee_linkage_offset_p2 = drawn_knee_linkage_offset + RotationY(pi/4)*rot*[A/2,0,0]';
    plot3([0,drawn_knee_linkage_offset(1),drawn_knee_linkage_offset_p1(1),drawn_knee_linkage_offset_p2(1)], ...
        [0,drawn_knee_linkage_offset(2),drawn_knee_linkage_offset_p1(2),drawn_knee_linkage_offset_p2(2)], ...
        [0,drawn_knee_linkage_offset(3),drawn_knee_linkage_offset_p1(3),drawn_knee_linkage_offset_p2(3)], ...
        "k-",LineWidth=1.5);
    plot3([knee_out(1),ankle_draw_offset(1)], ...
        [knee_out(2),ankle_draw_offset(2)], ...
        [knee_out(3),ankle_draw_offset(3)], ...
        "k-",LineWidth=1.5)

    axis equal
    axis([-30,140,-20,40,-140,40]);
    hold off

    CopyToOrthAxes(ax1, ax2);

    if write_video == 1
        drawnow
        frame = getframe(gcf);  % Get current frame
        writeVideo(v, frame);   % Write the frame to the video
    end
    pause(0.1)
end
writeVideo(v, repmat(frame,15,1));

if write_video == 1
    close(v)
end

function CopyToOrthAxes(from, to)
    objs = [findall(from, 'Type','Line');findall(from, 'Type','Quiver')];
    
    subplot(to)
    cla;
    copyobj(objs,to);
    axis equal
    axis([-30,140,-20,40,-140,40]);
    view(0,0)
    title("Side view")
end