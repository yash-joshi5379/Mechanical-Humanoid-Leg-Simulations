function out_theta = AngleOutAnkleLinkage(alpha, lengths)
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
    b_vec = p(:,3)-p(:,2);
    if imag(b_vec(3)) ~= 0 || imag(b_vec(1)) ~= 0
        out_theta = NaN;
    else
        out_theta = atan2(b_vec(3),b_vec(1));
    end
end


%%
clear;clc;close all

T = 5;
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

A = 50;
B = 15;
C = 50:5:120; %70
D = 40:10:70;

lengths(1:length(C),1:length(D)) = struct('A',A,'B',B,'C',NaN,'D',NaN);
for i=1:length(C)
    for j=1:length(D)
        lengths(i,j).C = C(i);
        lengths(i,j).D = D(j);
    end
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
theta_linkage = zeros([size(lengths,1),size(lengths,2),length(theta_out)]);

rotation_step_worm = theta_in(2) - theta_in(1);
rotation_step_spur = theta_out(2) - theta_out(1);

%% Plot
f = figure(WindowState="maximized");

for i = 1:n_steps
    for j=1:size(lengths,2)
        for k=1:size(lengths,1)
            theta_linkage(k,j,i) = AngleOutAnkleLinkage(theta_out(i)+deg2rad(linkage_angle_offset),lengths(k,j));
        end
    end
end


for j = 1:size(lengths,2)
    legend_ = strings(0,0);
    subplot(size(lengths,2),2,2*j-1);
    for i = 1:size(lengths,1)
        if all(isnan(theta_linkage(i,j,:)))
            continue
        end
        plot(theta_in,squeeze(theta_linkage(i,j,:)));
        legend_(i) = strcat("C = ",string(lengths(i,j).C));
        hold on
    end
    % legend(legend_);
    title(strcat("Input vs output angles - D = ",string(lengths(i,j).D)))
    xlabel('$\theta_{in}$ / rad','Interpreter','latex')
    ylabel('$\theta_{out}$ / rad','Interpreter','latex')
    hold off
    
    subplot(size(lengths,2),2,2*j);
    for i = 1:size(lengths,1)
        if all(isnan(theta_linkage(i,j,:)))
            continue
        end
        angles = squeeze(theta_linkage(i,j,:));
        angles = angles(~isnan(angles));
    
        plot([lengths(i,j).C,lengths(i,j).C],[0,rad2deg(abs(angles(end)-angles(1)))],'LineWidth',1.5); hold on
    end
    title(strcat("Angle range - D = ",string(lengths(i,j).D)))
    xlabel('C / mm','Interpreter','latex')
    ylabel('$\Delta\theta$ / $^\circ$','Interpreter','latex')
    hold off
end