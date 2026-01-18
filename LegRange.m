clc; clear;
% ang = linspace(0,2*pi,200);
% offset = deg2rad(-30:1:30);
% 
% l = 100;
% 
% for theta = offset
%     p = [zeros(size(ang));sin(ang);cos(ang)];
%     R = [cos(theta) 0 -sin(theta)
%          0          1 0
%          sin(theta) 0 cos(theta)];
%     p = R * p;
%     plot3(l*p(1,:),l*p(2,:),l*p(3,:),'b.'); hold on;
% end
% 
% ang = linspace(0,2*pi,100);
% for len = linspace(0.35*l,l,25)
%     p = [zeros(size(ang));sin(ang);cos(ang)];
%     R = [cos(offset(1)) 0 -sin(offset(1))
%          0          1 0
%          sin(offset(1)) 0 cos(offset(1))];
%     p = R * p;
%     plot3(len*p(1,:),len*p(2,:),len*p(3,:),'r.'); hold on;
% 
% 
%     p = [zeros(size(ang));sin(ang);cos(ang)];
%     R = [cos(offset(end)) 0 -sin(offset(end))
%          0          1 0
%          sin(offset(end)) 0 cos(offset(end))];
%     p = R * p;
%     plot3(len*p(1,:),len*p(2,:),len*p(3,:),'r.'); hold on;
% end

ang = linspace(pi/2,3*pi/2,200);
    p = [ones(size(ang));sin(ang);cos(ang)];
offset = deg2rad(-30:1:30);

l = 100;

for theta = offset
    plot3(l*sin(theta)*p(1,:),l*cos(theta)*p(2,:),l*cos(theta)*p(3,:),'b.'); hold on;

    if theta == offset(1) || theta == offset(end)
        for len = linspace(0.35*l,l,20)
            plot3(len*sin(theta)*p(1,:),len*cos(theta)*p(2,:),len*cos(theta)*p(3,:),'b.')
        end
    end
end



hold off;

axis equal
set(gca,'xtick',[],'xticklabel',[],'ytick',[],'yticklabel',[],'ztick',[],'zticklabel',[])