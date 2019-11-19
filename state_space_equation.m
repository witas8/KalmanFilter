clear all;

fps = 0.03 %frame per second

A = [1 fps 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 fps 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 fps;
     0 0 0 0 0 1];
 
B = [0; 0; 0; 0; -0.5*fps^2; -fps];

C = [1 0 1 0 1 0];

u = 9.81;

a = 52;
b = 45;
v0 = 8.3;
vx = v0*cosd(a)*sind(b);
vy = v0*cosd(a)*cos(b);
vz = v0*sin(a);
x0 = transpose([0 vx 0 vy 2 vz]); 
x(:, 1) = A*x0 + B*u;


N = 50
for k = 2:N
    x(:, k) = A*x(:, k-1) + B*u;
end


%graphs
t = 1:N;

%x
subplot(3, 2, 1);
plot(t, x(1, :))
xlabel('frames per second [1/s]');
ylabel('x coordinate [m]');
subplot(3, 2, 2);
plot(t, x(2, :))
xlabel('frames per second [1/s]');
ylabel('velocity in x axis [m/fps]');

%y
subplot(3, 2, 3);
plot(t, x(3, :))
xlabel('frames per second [1/s]');
ylabel('y coordinate [m]');
subplot(3, 2, 4);
plot(t, x(4, :))
xlabel('frames per second [1/s]');
ylabel('velocity in y axis [m/fps]');

%z
subplot(3, 2, 5);
plot(t, x(5, :))
xlabel('frames per second [1/s]');
ylabel('z coordinate [m]');
subplot(3, 2, 6);
plot(t, x(6, :))
xlabel('frames per second [1/s]');
ylabel('velocity in z axis [m/fps]');

%prepare for Kalman
X = transpose(x(1,:));
Vx = transpose(x(2,:));
Y = transpose(x(3,:));
Vy = transpose(x(4,:));
Z = transpose(x(5,:));
Vz = transpose(x(6,:));