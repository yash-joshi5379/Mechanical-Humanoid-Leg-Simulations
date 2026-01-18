function a = Cross(u,v)
    a = [u(2)*v(3) - u(3)*v(2)
         -u(1)*v(3) + u(3)*v(1)
         u(1)*v(2) - u(2)*v(1)];
end
function mid = Midpoint(p1,p2)
    mid = [(p1(1)+p2(1)),(p1(2)+p2(2)),(p1(3)+p2(3))] / 2;
end

function varargout = DrawAnkleLinkage(alpha, lengths, rot, offset, draw, debugInfo)
    % Calculate the angle theta, based on the given angles and lengths
    k = (lengths.C^2 - lengths.A^2 - lengths.B^2 - lengths.D^2) + 2*lengths.A*lengths.B*cos(alpha);
    a = lengths.A - lengths.B*cos(alpha);
    b = lengths.B*sin(alpha);
    
    theta = asin(k ./ (2*lengths.D*sqrt( (a.^2) + (b.^2) ))) + atan2(b,a);

    % Determine the end points of the linkages
    p = [-lengths.B*sin(alpha), -lengths.D*cos(theta),           0
          0,                     0,                              0
         -lengths.B*cos(alpha), -lengths.A-lengths.D*sin(theta),-lengths.A];

    % Rotate the points according to the provided matrix, and plot
    p(:,1) = rot*(p(:,1)+offset);
    p(:,2) = rot*(p(:,2)+offset);
    p(:,3) = rot*(p(:,3)+offset);
    set(draw(1),'XData',p(1,:),'YData',p(2,:),'ZData',p(3,:))
    out_p = p(:,2);

    b_vec = p(:,3)-p(:,2);
    % disp(norm(p(:,3)-p(:,2)))
    local_up = rot * [0,-1,0]';

    norm_vec = 10*Cross(b_vec/norm(b_vec),local_up);
    out_theta = atan2(b_vec(3),b_vec(1));
    if debugInfo == 1
        m = Midpoint(p(:,3),p(:,2));
        set(draw(2),'XData',m(1),'YData',m(2),'ZData',m(3),'UData',norm_vec(1),'VData',norm_vec(2),'WData',norm_vec(3))
    end

    varargout = {norm_vec,out_theta,out_p};
end


%%
clear;clc;close all

T = 2.5;
time_step = 0.01;
n_steps = T/time_step;

%% Worm gear
worm_pitch = 2;
worm_length = 40;
worm_radius = 10;
worm_n_plot = 500;

worm_centre = [0,0,0]';
worm_axis = [1,0,0]';

rotation_amount = 12*pi;

%% Spur gear
spur_plot = stlread("Gear_m2_n24.stl").Points;
spur_pitch = 2;
spur_n = 24;
spur_radius = spur_pitch*spur_n;

spur_centre = [0,0,-spur_radius/2 - worm_radius]';
spur_axis = [0,1,0]';

%% Linkage
linkage_angle_offset = 140;

% lengths = struct('A',50,'B',15,'C',75,'D',30);
% lengths = struct('A',50,'B',15,'C',105,'D',60);
lengths = struct('A',50,'B',15,'C',90,'D',50);

rot = [1 0 0
       0 1 0
       0 0 1];

%%
write_video = 0;

if write_video == 1
    frame_rate = 1 / time_step;
    v = VideoWriter('Sim_WormLinkage.mp4', 'MPEG-4');  % Create VideoWriter object for MP4 file
    v.FrameRate = frame_rate;  % Set the frame rate of the video
    % Open the video file
    open(v);
end

%% Pre-calculations
% Worm
% Number of times the thread wraps the shaft of the worm gear
n_rots = worm_length / (pi*worm_pitch);
theta_worm_plot = linspace(0,2*pi*n_rots,worm_n_plot);

% 1 rotation of input shaft = 1 tooth advance on driven gear, therefore
% ratio is 1/spur_n
ratio_worm_spur = -1/spur_n;

% Gear graphs
theta_in = linspace(0,rotation_amount,n_steps);
theta_out = theta_in.*ratio_worm_spur;
theta_linkage = zeros([1,length(theta_out)]);

w_in = rotation_amount/T * ones([1,length(theta_out)]);
w_spur = w_in*ratio_worm_spur;
w_linkage = NaN([1,length(theta_out)]);

rotation_step_worm = theta_in(2) - theta_in(1);
rotation_step_spur = theta_out(2) - theta_out(1);

p1 = lengths.B*[-sin(theta_out+deg2rad(linkage_angle_offset));-cos(theta_out+deg2rad(linkage_angle_offset))];
traj = zeros([3,n_steps]);

%% Plot
f = figure(WindowState="maximized");
ax1 = subplot(2,3,[1,4]);
    title(ax1, "Model")
ax2 = subplot(2,3,[2,5]);
ax3 = subplot(2,3,3);
ax4 = subplot(2,3,6);
title(ax4,"Worm gear against output angle")

subplot(ax1);
worm = plot3(linspace(0,worm_length,worm_n_plot)+worm_centre(1)-worm_length/2, ...
    worm_radius*cos(theta_worm_plot)+worm_centre(2), ...
    worm_radius*sin(theta_worm_plot)+worm_centre(3),".");
hold on
spur = plot3(spur_plot(:,1)+spur_centre(1),zeros(size(spur_plot(:,1)))+spur_centre(2),spur_plot(:,2)+spur_centre(3),'.');
point = plot3(p1(1,1)+spur_centre(1),spur_centre(2),p1(2,1)+spur_centre(3),"o");
link = plot3(0,0,0,'k-o','LineWidth',2,'MarkerFaceColor','r');
trajectory = plot3(0,0,0,'g--');
link_quiver = quiver3(0,0,0,0,0,0,"r",'LineWidth',1.5,'ShowArrowHead','on','MaxHeadSize',1.5);
axis equal
axis([-50,35,-20,20,-150,20])

subplot(ax3)
    ax3_plot1 = plot(0,0,'r-'); hold on;
    ax3_plot2 = plot(0,0,'b-'); hold off;
    axis(ax3,[0,theta_in(end),-1.75,1.5]);
    title(ax3,"Linkage angles");
    legend(ax3,'Spur Gear','Linkage Output Arm')
    xlabel(ax3,'$\theta_{in}$ / rad','Interpreter','latex');
    ylabel(ax3,'$\theta_{out}$ / rad','Interpreter','latex');

subplot(ax4)
    ax4_plot1 = plot(0,0,'r-'); hold on;
    ax4_plot2 = plot(0,0,'b-');
    ax4_plot3 = plot(0,0,'g-'); hold off;
    title(ax4,"Angular velocities");
    xlim(ax4,[0,theta_in(end)]);
    legend(ax4,'Worm Gear','Spur Gear', 'Linkage Output Point')
    xlabel(ax4,'$\theta_{in}$ / rad','Interpreter','latex');
    ylabel(ax4,'$\omega$ / $rads^{-1}$','Interpreter','latex');

pause(1)
for i = 1:n_steps
    subplot(ax1);
    rotate(worm,worm_axis,rad2deg(rotation_step_worm),worm_centre);
    rotate(spur,spur_axis,rad2deg(rotation_step_spur),spur_centre);
    set(point, 'XData',p1(1,i)+spur_centre(1),'ZData',p1(2,i)+spur_centre(3));
    [out_vec, out_theta, out_p] = DrawAnkleLinkage(theta_out(i)+deg2rad(linkage_angle_offset),lengths,rot,spur_centre,[link,link_quiver],1);
    theta_linkage(i) = out_theta;
    traj(:,i) = out_p;
    if i>1
        w_linkage(i) = norm(traj(:,i)-traj(:,i-1)) / time_step / lengths.A;
    end
    set(trajectory, 'XData',traj(1,1:i), 'YData',traj(2,1:i), 'ZData',traj(3,1:i))

    subplot(ax2); cla; 
    title(ax2, "Front view")
    copyobj([worm,spur,point,link,link_quiver,trajectory],ax2);
    axis equal; 
    axis([-50,35,-20,20,-150,20])
    view(0,0)

    set(ax3_plot1,'XData',theta_in(1:i),'YData',theta_out(1:i));
    set(ax3_plot2,'XData',theta_in(1:i),'YData',theta_linkage(1:i));

    set(ax4_plot1, 'XData',theta_in(1:i),'YData',w_in(1:i));
    set(ax4_plot2, 'XData',theta_in(1:i),'YData',w_spur(1:i));
    set(ax4_plot3, 'XData',theta_in(1:i),'YData',w_linkage(1:i));


    if write_video == 1
        drawnow
        frame = getframe(gcf);  % Get current frame
        writeVideo(v, frame);   % Write the frame to the video
    end

    pause(time_step)
end
if write_video
    writeVideo(v, repmat(frame,15,1));
    close(v)
end