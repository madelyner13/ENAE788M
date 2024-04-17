%% rosbag plotter

close all;
clear;
clc;

% curr_path = pwd;
% bag_path = sprintf('%s/rosbag2_2024_03_27-00_57_07/rosbag2_2024_03_27-00_57_07_0.db3',curr_path);
% bag = ros2bagreader(bag_path);
% 
% local_pos = select(bag,"Topic","/fmu/out/vehicle_local_position");
% local_pos = readMessages(local_pos);

load('local_pos.mat');

x = zeros(1,size(local_pos,1));
y = zeros(1,size(local_pos,1));
z = zeros(1,size(local_pos,1));
heading = zeros(1,size(local_pos,1));
time = zeros(1,size(local_pos,1));
for k = 1:size(local_pos,1)
    x(k) = local_pos{k}.x;
    y(k) = local_pos{k}.y;
    z(k) = local_pos{k}.z;
    heading(k) = local_pos{k}.heading;
    time(k) = local_pos{k}.timestamp-local_pos{1}.timestamp;
end

[headingx,headingy] = pol2cart(heading,0.1);
headingvec = [headingx'+x',headingy'+y',z'];

figure()
plot(time,x)
grid on
title('X-Position over Time')
xlabel('Time (ms)')
ylabel('X-Position (m)')

figure()
plot(time,y)
grid on
title('Y-Position over Time')
xlabel('Time (ms)')
ylabel('Y-Position (m)')

figure()
plot(time,z)
grid on
title('Z-Position over Time')
xlabel('Time (ms)')
ylabel('Z-Position (m)')

figure()
plot3(x,y,z)
grid on
title('3D-Position')
xlabel('X-Position (m)')
ylabel('Y-Position (m)')
zlabel('Z-Position (m)')
axis equal

figure()
plot(x,y)
grid on
title('XY-Position')
xlabel('X-Position (m)')
ylabel('Y-Position (m)')

% v = VideoWriter('traj_tracking','Motion JPEG AVI');
% v.Quality = 95;
% open(v);
% figure()
% set(gcf,'Position',[10 10 1500 900]);
% plot3(x,y,z,'b')
% hold on
% grid on
% axis equal
% xlabel('X-Position (m)')
% ylabel('Y-Position (m)')
% zlabel('Z-Position (m)')
% title('Trajectory Tracking Demonstration')
% for k=1:length(x)
%     pplot = plot3(x(k),y(k),z(k),'r.','MarkerSize',10);
%     hplot = plot3([x(k) headingvec(k,1)],[y(k) headingvec(k,2)],[z(k) headingvec(k,3)],'r','LineWidth',1.5);
%     legend('Trajectory','Drone Position','Heading')
%     MM = getframe(gcf);
%     writeVideo(v,MM);
%     delete(pplot);
%     delete(hplot);
% end
% close(v);
% close(gcf);

