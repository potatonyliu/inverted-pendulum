function dzdt = cart_pendulum(t, z, K, P)
    % z = [x; xdot; phi; phidot]
    
    M = P(1);
    m = P(2);
    r = P(3);
    g = P(4);


    x = z(1);
    xdot = z(2);
    phi = z(3);
    phidot = z(4);
  

    F = -K * z;


    % dzdt = [z(2); (g*m*phi+F)/M; z(4); (F+(M+m)*g*phi)/(r*M)];
    dzdt = [z(2); (-g*m*sin(phi)*cos(phi)+F+m*r*sin(phi)*phidot^2)/((M+m)-m*cos(phi)^2); z(4); (cos(phi)*F+(m*r*cos(phi)*sin(phi)*phidot^2)+(M+m)*g*sin(phi))/(r*((M+m)-m*cos(phi)^2))];
end
i = 1;

figure;
subplot(2,1,1); hold on;
subplot(2,1,2); hold on;

for angle = 0.01:0.01:0.3
    t = 10;
    z0 = [0, 0, angle, 0];
    
    M = 1.486;
    m = 0.06315;
    r = 0.528;
    g = 9.81;
    
    A = [0 1 0 0; 0 0 g*m/M 0; 0 0 0 1; 0 0 (M+m)*g/(r*M) 0];
    B = [0; 1/M; 0; 1/(r*M)];
    Q = diag([10,1,1000,4]);
    R = 0.1;
    
    K = lqr(A, B, Q, R);
    K
    P = [M, m, r, g];
    
    tspan = [0 7];
    odefun = @(t,z) cart_pendulum(t,z, K, P);
    [T, Z] = ode45(odefun, tspan, z0);
    % Plot
    subplot(2,1,1);
    plot(T, Z(:,1));
    ylabel('Cart position (m)');
    
    subplot(2,1,2);
    plot(T, Z(:,3)*180/pi);
    ylabel('Angle (degrees)');
    xlabel('Time (s)');
        
    max_forces(i) = max(abs(-K*Z'));
    max_velocities(i) = max(abs(Z(:,2)));
    angles(i) = angle*180/pi;
    i = i + 1;
end

T_results = table(angles', max_forces', max_velocities', 'VariableNames', {'Angle (degree)', 'Max Force (N)', 'Max Velocity (m/s)'});
disp(T_results);
figure;
subplot(2,1,1);
plot(angles'*180/pi, max_forces');
ylabel('Max Forces (N)');
subplot(2,1,2);
plot(angles'*180/pi, max_velocities');
ylabel('Max Velocities (m/s)');
xlabel('Angle (degrees)');
