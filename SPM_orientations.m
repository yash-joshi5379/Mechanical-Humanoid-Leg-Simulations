clc; clear; close all;
home_core_ends = [14.2430    4.9711  -19.2142
                  13.9634  -19.3165    5.3531
                 -10.4120  -10.4120  -10.4120];

%       x y z
axes = [1 0 0
        0 1 0
        0 0 1];
home_pos = [1,1,-1;1,-1,1;-1,-1,-1];
perms = table2array(combinations([1,-1],[1,-1],[1,-1],[1,-1],[1,-1],[1,-1],[1,-1],[1,-1],[1,-1]));

theta = deg2rad([65,75,90]);

% plot3(0,0,0,"x b"); hold on;
% valid = RenderSPM(theta, axes, home_core_ends, axes, 1, [], [], 0, home_pos_override=home_pos, validate=1);
% axis equal
% set(gcf().Children, 'xlim',[-30,30],'ylim',[-30,30],'zlim',[-30,20]);
% return
figure()
n = 1;
% for i = 1:size(perms,1)
perms_index = [1,2,3,6,10,11,16,30];
for i = 1:length(perms_index)
    subplot(2,4,i);
    home_pos_override = [perms(perms_index(i),1:3);perms(perms_index(i),4:6);perms(perms_index(i),7:9)];
    plot3(0,0,0,"x b"); hold on;
    
    valid = RenderSPM(theta, axes, home_core_ends, axes, 1, [], [], 0, home_pos_override=home_pos_override, validate=1);
    if valid == 0
        hold off; plot3(0,0,0,"x b");
        continue;
    end
    axis equal
    set(gcf().Children, 'xlim',[-30,30],'ylim',[-30,30],'zlim',[-35,20]);
    % print([pwd '/orientations/',num2str(n)],'-dpng','-r300')
    % n = n + 1;
    hold off
end