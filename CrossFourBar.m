function mid = Midpoint(p1,p2)
    mid = [(p1(1)+p2(1)) / 2,(p1(2)+p2(2)) / 2];
end

clear; clc;

A = 20;
B = 10;
C = 25;
D = 30;

% A = 20;
% B = 10;
% C = 10;
% D = 4;

% (C + D - A) / B < 1;
% (C - D) / B < 1

% alpha = linspace(deg2rad(55),deg2rad(50)+(pi/3),100);
alpha = linspace(0,deg2rad(50)+(pi/3),100);
% alpha = linspace(deg2rad(55),deg2rad(97),100);
dim = ones([1,length(alpha)]);

k = (B.^2 - A.^2 - C.^2 - D.^2)*dim + 2*A*C*cos(alpha);
a = -A*dim + C*cos(alpha);
b = -C*sin(alpha);

theta = acos(k ./ (2*D*sqrt( (a.^2) + (b.^2) ))) + atan2(b,a);
theta(imag(theta)~=0) = NaN;

traj = zeros([2,length(theta)]);
end_angle = atan2((D*sin(theta)-C*sin(alpha)),(A-D*cos(theta)-C*cos(alpha)));
end_angle = end_angle + 2*pi*(end_angle<0) - pi*ones([1,length(end_angle)]);

figure(WindowState="maximized")
ax_1 = subplot(2,3,[1,2,4,5]);
ax_2 = subplot(2,3,3);
    angle_plot = plot(0,0);
    title(ax_2,"Linkage angles");
    xlabel(ax_2,'$\theta_{in}$ / rad','Interpreter','latex')
    ylabel(ax_2,'$\theta_{out}$ / rad','Interpreter','latex')
    xlim(ax_2,[alpha(1),alpha(end)]);
    ylim(ax_2,[min(end_angle),max(end_angle)])
ax_3 = subplot(2,3,6);
    w_plot_input = plot(0,0,'r-'); hold on;
    w_plot_linkage = plot(0,0,'b-'); hold off;
    title(ax_3,"Angular velocities");
    xlim(ax_3,[alpha(1),alpha(end)]);
    % ylim([0,0.45])
    legend(ax_3,'Input','Output')
    xlabel(ax_3,'$\theta_{in}$ / rad','Interpreter','latex');
    ylabel(ax_3,'$\omega$ / $rads^{-1}$','Interpreter','latex');

steps = [1:length(alpha)]; % ,length(alpha)-1:-1:1

time_step = 1/30;
w_in = (alpha(2) - alpha(1))/time_step * ones([1,length(end_angle)]);
w_linkage = NaN([1,length(end_angle)]);

%%
write_video = 0;

if write_video == 1
    frame_rate = 1/time_step;
    v = VideoWriter('Sim_Cross4Bar.mp4', 'MPEG-4');  % Create VideoWriter object for MP4 file
    v.FrameRate = frame_rate;  % Set the frame rate of the video
    % Open the video file
    open(v);
end

% pointsAreCollinear = @(xy) rank(xy(2:end,:) - xy(1,:)) == 1;

%%
start = -1;
for i=steps
    subplot(ax_1);
    if isnan(theta(i))
        continue;
    end
    if start == -1
        start = i;
    end
    p_x = [0,C*cos(alpha(1)),A-D*cos(theta(1)),A];
    p_y = [0,C*sin(alpha(1)),D*sin(theta(1)),0];
    plot(p_x, p_y,'b--','LineWidth',1,'MarkerFaceColor','r'); hold on

    p_x = [0,C*cos(alpha(i)),A-D*cos(theta(i)),A];
    p_y = [0,C*sin(alpha(i)),D*sin(theta(i)),0];
    plot(p_x, p_y,'k-o','LineWidth',2,'MarkerFaceColor','r');

    traj(:,i) = Midpoint([p_x(2),p_y(2)], [p_x(3),p_y(3)]);

    if i>1
        w_linkage(i) = norm(traj(:,i)-traj(:,i-1)) / time_step / norm(traj(:,i));
    end

    plot(traj(1,start:i),traj(2,start:i),"--g")
    axis equal
    axis([-20,30,-2,35]);
    hold off

    set(angle_plot,'XData',alpha(1:i),'YData',end_angle(1:i));

    set(w_plot_input,'XData',alpha(start+1:i),'YData',w_in(start+1:i));
    set(w_plot_linkage,'XData',alpha(start+1:i),'YData',w_linkage(start+1:i));

    if write_video == 1
        drawnow
        frame = getframe(gcf);  % Get current frame
        writeVideo(v, frame);   % Write the frame to the video
    end
    pause(0.01);
end

if write_video == 1
    writeVideo(v, repmat(frame,15,1));
end

if write_video == 1
    close(v)
end