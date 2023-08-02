% Choose directory
rootdir = ['D1215B1_5F4/run1','D1215B1_5F4/run2','D1215B1_5F4/run3'];
%Load Data

for a=1:16:48
    DesiredEndEffectorVelocity = load(fullfile(rootdir(a:a+15),"desiredEndEffectorVelocity.csv"));
    DesiredJointPosition = load(fullfile(rootdir(a:a+15),"desiredJointPosition.csv"));
    JointPosition = load(fullfile(rootdir(a:a+15),"jointPosition.csv"));
    JointVelocity = load(fullfile(rootdir(a:a+15),"jointVelocity.csv"));

    Time = load(fullfile(rootdir(a:a+15),"simulationTime.csv"));
    x = zeros(length(Time),3);
    xDot = zeros(length(Time)-1,3);
    xpDot =zeros(length(Time)-1,7);

    for i =1:length(Time)
        [x(i,:), ~] = forwardKinematics(JointPosition(i,:));
    end
    for i =1:length(x)-1
        xDot(i,:) =  (x(i+1,:) - x(i,:)) / (Time(i+1)-Time(i));
    end
    for i =1:length(x)-1
        xpDot(i,:) =  (JointPosition(i+1,:) - JointPosition(i,:)) / (Time(i+1)-Time(i));
    end
    % Desired Path and Derivative of Path
    radius = 0.12;
    line_length = 0.15;
    alpha = (2*line_length)/(radius*pi) + 1;

    % Time vector
    sspace = linspace(0, alpha + 1, length(Time));  % Generate 100 points within the total time

    P_z = zeros(1,length(sspace));
    P_x = zeros(1,length(sspace));
    P_z_derivative = zeros(1,length(sspace));
    P_x_derivative = zeros(1,length(sspace));

    for i=1:length(sspace)
        s=sspace(i);
        if s <= 1
            % Calculate x and y coordinates of the first circle segment
            P_z(i) = -radius * cos(s*pi/2);
            P_x(i) = -radius * sin(s*pi/2);

            P_z_derivative(i) = radius * (pi/2) * sin(s * pi/2);
            P_x_derivative(i) = -radius * (pi/2) * cos(s * pi/2);

        elseif s <= alpha && s>1
            % Calculate the coordinates of the straight line segment
            P_z(i) = (s-1) * (radius * pi) / 2;
            P_x(i) = -radius * ones(1);

            P_z_derivative(1,i) = radius * pi / 2;
            P_x_derivative(1,i) = 0;
        else
            P_z(i) = radius * sin((s-alpha) * pi/2) + line_length;
            P_x(i) = -radius * cos((s-alpha) * pi/2);
            P_z_derivative(i) = radius * pi/2 * cos((s-alpha) * pi/2);
            P_x_derivative(i) = radius * pi/2 * sin((s-alpha) * pi/2);
        end

    end


    % Plotting
    figure(1)
    hold on;   grid on
    plot(x(:,3),-x(:,1),'.')
    if a == 33
        plot(P_z + 0.515509+radius,P_x + 0.0210774,'LineWidth',1.5)
    end
    xlabel('z direction (m)');    ylabel('x direction (m)')
    title('Desired and Actual Position')
    legend('Actual Path1', 'Actual Path2', 'Actual Path3','Desired Path' )
    axis('equal');

    figure(2)
    hold on;    grid on
    plot(Time(10:end-1),xDot(10:end,1),'.')
    if a == 33
        plot(Time,DesiredEndEffectorVelocity(:,1),'LineWidth',1.5)
    end
    xlabel('Time (s)');    ylabel('Velocity (m/s)')
    title('Velocity in X direction')
    legend('Actual Velocity1','Actual Velocity2','Actual Velocity3','Desired Velocity')

    figure(3)
    hold on;    grid on
    plot(Time(10:end-1),xDot(10:end,3),'.')
    if a==33
        plot(Time,DesiredEndEffectorVelocity(:,3),'LineWidth',1.5)
    end
    xlabel('Time (s)');    ylabel('Velocity (m/s)')
    title('Velocity in Z direction')
    legend('Actual Velocity1','Actual Velocity2','Actual Velocity3','Desired velocity')

end

s = zeros(length(Time),1);
sDot = zeros(length(Time),1);
radius = 0.12;
line_length = 0.15;
tBlend = 1.5;
tFinal = 4;

alpha = (2 * line_length) / (radius * pi) + 1;
W = (alpha + 1) / (tBlend * (tFinal - tBlend));
for i = 1:length(sDot)
    t = i * tFinal / length(sDot);
    if  t <= tBlend
        s(i) = (W * t^2) / 2;
        sDot(i) = W * t;
   
   
    elseif t > tBlend && t < tFinal - tBlend
        s(i) = W * tBlend * (t - tBlend) + ((W * tBlend^2))/2;
        sDot(i) = W * tBlend;

       
    else
        s(i) =  W * (tFinal) - (W  * tFinal^2 / 2) + (W * tFinal * t) - (W * t^2 / 2) ;
        sDot(i) = W * ( tFinal -t);
    end
       
end

figure(4)
hold on;    grid on
plot(Time(2:end),xpDot(:,2),'.', Time,JointVelocity(:,2),LineWidth=2)

xlabel('Time (s)'); ylabel('Velocity degree/s'); title('2nd Joints Velocity')
legend('Calculated Velocity', 'Reported Velocity')

figure(5)
hold on;    grid on
plot(Time(2:end),xpDot(:,4),'.', Time,JointVelocity(:,4),LineWidth=2)
xlabel('Time (s)'); ylabel('Velocity degree/s'); title('4th Joints Velocity')
legend('Calculated Velocity', 'Reported Velocity')

figure(6)
hold on;    grid on
plot(Time(2:end),xpDot(:,6),'.', Time,JointVelocity(:,6),LineWidth=2)
xlabel('Time (s)'); ylabel('Velocity degree/s'); title('6th Joints Velocity')
legend('Calculated Velocity', 'Reported Velocity')

figure(7)
hold on;    grid on
quiver(x(2:end,3),-x(2:end,1),xDot(:,3),xDot(:,1))
xlabel('Time (s)'); ylabel('Velocity degree/s'); title('6th Joints Velocity')
legend('Calculated Velocity', 'Reported Velocity')

figure(8)

subplot(321)
plot(P_z,P_x)
xlabel('Time (s)'); ylabel('Position (m)')
title('Desired Trajectory')

subplot(322)
plot(Time, P_x_derivative)
xlabel('Time (s)'); ylabel('direction')
title('derivative of position in x direction Trajectory')


subplot(323)
plot(Time, P_z_derivative)
xlabel('Time (s)'); ylabel('direction')
title('derivative of position in z direction Trajectory')

subplot(324)
plot(Time,sDot)
xlabel('Time (s)'); ylabel('direction')
title('derivative of s function')



subplot(325)
plot(Time,P_x_derivative.*sDot' )
xlabel('Time (s)'); ylabel('direction')
title('P_x_derivative x sDot')


subplot(326)
plot(Time,P_z_derivative.*sDot'  )
xlabel('Time (s)'); ylabel('direction')
title('P_z_derivative x sDot')


