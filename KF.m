clear all;

%motion data measurements
meas = load('camera_meas_xyz.txt');
x_meas = meas(:,1) + 0.4*rand(size(meas(:,1)));
y_meas = meas(:,3) + 0.3*rand(size(meas(:,1)));
z_meas = meas(:,5) + 0.2*rand(size(meas(:,1)));

%time vector
fps = 0.03; %frame per second
t = fps:size(meas, 1) 

%state space model
A = [1 fps 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 fps 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 fps;
     0 0 0 0 0 1];
 
B = [0; 0; 0; 0; -0.5*fps^2; -fps];

C = [1 0 0 0 0 0; 
    0 0 0 0 0 0; 
    0 0 1 0 0 0; 
    0 0 0 0 0 0; 
    0 0 0 0 1 0; 
    0 0 0 0 0 0];

%control signal
u = 9.81; %gravitational acceleration

%depth range resolution
dd = 1; %disparity delta
f = 650; %focal length
T = 0.07; %baseline [7cm]

%starting point
a = 52; %alpha launch angle
b = 45; %beta approach angle 
v0 = 8.3; %initial velocity
vx = v0*cosd(a)*sind(b);
vy = v0*cosd(a)*cos(b);
vz = v0*sin(a);
x_prio = [0; 0; 0; 0; 0; 0];
x_post = transpose([0 vx 0 vy 2 vz]); %states initialization
P_prio = zeros(6,6);
P_post = zeros(6,6);

%results
x_out = zeros(1, size(t,2));
y_out = zeros(1, size(t,2));
z_out = zeros(1, size(t,2));


for i = 1:size(meas, 1); 
    V = diag([0.2 0.2 0.2 0.2 0.2 0.2]); %disturbances 
    dz = meas(i,5)^2*dd/f/T;
    W = diag([0.5*dz 0.1*dz 0.5*dz 0.1 dz 0.1]); %noise 
    
    %prediction
    x_prio = A*x_post + B*u; %A =[6x6] dlatego x=[6x1], a nie y=[3x1] (xyz)
    P_prio = A*P_post*A + V;
    
    %update
    e = meas(i,1:6)' - C*x_prio;
    S = C*P_prio*C' + W;
    K = P_prio*C'*S;
    x_post = x_prio + K*e;
    P_post = P_prio - K*S*K';
    
    %results
    x_out(i) = x_post(1);
    y_out(i) = x_post(3);
    z_out(i) = x_post(5);
end

%x
figure
plot(t, x_meas, 'b', t, x_out, 'r', 'LineWidth', 1.5)
xlabel('time [1/s]')
ylabel('x coordinate [m]')
legend('measurement', 'estimation')

%y
figure
plot(t, y_meas, 'b', t, y_out, 'r', 'LineWidth', 1.5)
xlabel('time [1/s]')
ylabel('y coordinate [m]')
legend('measurement', 'estimation')

%z
figure
plot(t, z_meas, 'b', t, z_out, 'r', 'LineWidth', 1.5)
xlabel('time [1/s]')
ylabel('z coordinate [m]')
legend('measurement', 'estimation')