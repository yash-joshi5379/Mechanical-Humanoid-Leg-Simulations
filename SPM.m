clc; clear; close all;
home_core_ends = [   17.4082    6.0758  -23.4840
                     17.0664  -23.6091    6.5427
                    -12.7258  -12.7257  -12.7258];

%       x y z
axes = [1 0 0
        0 1 0
        0 0 1];

angles = deg2rad(-30:0.5:30);


f = figure(); %WindowState="maximized"

ax_sim = subplot(3,2,[1,3,5]);
ax_roll = subplot(3,2,2);
    roll_plot = plot(0,0,'r');
    axis([angles(1),angles(end),-pi/2,pi/2])
    title('Roll')
    xlabel('$\theta_{in}$ / rad','Interpreter','latex');
    ylabel('$\theta$ / rad','Interpreter','latex');
ax_pitch = subplot(3,2,4);
    pitch_plot = plot(0,0,'g');
    axis([angles(1),angles(end),-pi/2,pi/2])
    title('Pitch')
    xlabel('$\theta_{in}$ / rad','Interpreter','latex');
    ylabel('$\theta$ \ rad','Interpreter','latex');
ax_yaw = subplot(3,2,6);
    yaw_plot = plot(0,0,'b');
    axis([angles(1),angles(end),-pi/2,2*pi])
    title('Yaw')
    xlabel('$\theta_{in}$ / rad','Interpreter','latex');
    ylabel('$\theta$ \ rad','Interpreter','latex');

%%
write_video = 0;

if write_video == 1
    frame_rate = 30;  % Frame rate is the inverse of the time step
    videoFile = 'Sim_SPM.mp4';  % Define the video file name
    v = VideoWriter(videoFile, 'MPEG-4');  % Create VideoWriter object for MP4 file
    v.FrameRate = frame_rate;  % Set the frame rate of the video
    % Open the video file
    open(v);
end

%%
pause(1)
for ax=1:4
    path = [];
    data = [];

    last_yaw = NaN;
    frameArray = repmat(getframe(gcf), length(angles), 1);

    for i=1:length(angles)
        angle = angles(i);
        subplot(ax_sim)
        plot3(0,0,0,"x b"); hold on;

        if ax == 4
            theta = [angle,angle,angle];
        else
            theta = [0,0,0];
            theta(ax) = angle;
        end

        [path,data,end_rot_m] = RenderSPM(theta, axes, home_core_ends, axes, 1, path, data, last_yaw);
        last_yaw = data(end,1);
        set(ax_sim,'xlim',[-30,30],'ylim',[-30,30],'zlim',[-35,20],'DataAspectRatio',[1,1,1]);

        p = end_rot_m * [0,0,20]';
        plot3([path(1,end),p(1)],[path(2,end),p(2)],[path(3,end),p(3)]);
        hold off

        set(roll_plot,'XData',angles(1:size(data,1)),'YData',data(:,3))
        set(pitch_plot,'XData',angles(1:size(data,1)),'YData',data(:,2))
        set(yaw_plot,'XData',angles(1:size(data,1)),'YData',data(:,1))

        if write_video == 1
            drawnow
            frameArray(i) = getframe(gcf);  % Get current frame
        end

        pause(0.1);
    end

    if write_video == 1
        disp("Writing")
        writeVideo(v, frameArray);   % Write the frame to the video
        disp("Written")
        frameArray = repmat(getframe(gcf), 60, 1);
        writeVideo(v, frameArray);
    end
    pause(2);
end
if write_video == 1
    writeVideo(v, repmat(frame,15,1));
    close(v);
end