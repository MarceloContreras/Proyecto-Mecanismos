clc;clear all;close all
points=xlsread('PointsFinal.xlsx');
x=points(:,2)/100;
y=points(:,3)/100;
figure(1)
plot(x,y),xlim([-0.4,0.6]),ylim([-1,0.1]),xlabel('x(cm)'),ylabel('y(cm)'),title('Trayectoria deseada')
hold on;
grid on;
x=x-0.65;
y=y-1.16;
plot(x,y),xlim([-0.4,0.6]),ylim([-1,0.1])
grid on;
