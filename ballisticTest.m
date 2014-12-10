% close all
clear, clc

p0 = [0,0,5];
azimuth = pi/4;
elevation = pi/3;
velocity = 5;

% for testing: theta = azimuth, phi = elevation

%%
[T,P] = ballistic(p0,azimuth,elevation,velocity);
velVect = [p0;p0+velocity.*[cos(azimuth)*cos(elevation),sin(azimuth)*cos(elevation),sin(elevation)]];

figure(1)
plot3(velVect(:,1),velVect(:,2),velVect(:,3),'r-.') % plot v0 unit vector
hold on
plot3(P(:,1),P(:,2),P(:,3)) % plot trajectory
scatter3(P(end,1),P(end,2),0,100,'rx') % plot endpoint
axis equal
scale = axis;
scale(5) = 0;
scale(2) = scale(2)*1.25;
scale(4) = scale(4)*1.25;
axis(scale)
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
view(90-37.5,30)
grid on
hold off


figure(2)
subplot(3,1,1)
xplot = plotyy(T,P(:,4),T,P(:,7));
ylabel(xplot(1),'velocity (m/s)')
ylabel(xplot(2),'acceleration (m/s^2)')
title('x-direction')
grid on

subplot(3,1,2)
yplot = plotyy(T,P(:,5),T,P(:,8));
ylabel(yplot(1),'velocity (m/s)')
ylabel(yplot(2),'acceleration (m/s^2)')
title('y-direction')
grid on

subplot(3,1,3)
zplot = plotyy(T,P(:,6),T,P(:,9));
xlabel('time (s)')
ylabel(zplot(1),'velocity (m/s)')
ylabel(zplot(2),'acceleration (m/s^2)')
title('z-direction')
grid on
