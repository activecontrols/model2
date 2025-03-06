clear; close all;

C_vert = [0 0 1; 1 0 0; 0 1 0];
q0 = dcm2quat(C_vert).';
omega0 = [0; 0; 0];

t_list = linspace(0, 10, 10000);
[tout, x] = ode45(@(t,x) odefun(t,x), t_list, [x0; v0; q0; omega0]);

plot(tout, x(:, 7:13))
legend('q_0', 'q_1', 'q_2', 'q_3', 'omega_1', 'omega_2', 'omega_3')
grid on

function dxdt = odefun(t, x)
    theta = 0;
    phi = 0;
    thrust = 100;
    tau = 0;
    rTB = 1;
    J1_1 = 100;
    J1_2 = 0;
    J1_3 = 0;
    J2_1 = 0;
    J2_2 = 500;
    J2_3 = 0;
    J3_1 = 0;
    J3_2 = 0; 
    J3_3 = 500; 
    m = 10;
    g = 9.81;
    v1 = x(4);
    v2 = x(5);
    v3 = x(6);
    q0 = x(7);
    q1 = x(8);
    q2 = x(9);
    q3 = x(10);
    omega1 = x(11);
    omega2 = x(12);
    omega3 = x(13);


    dxdt = zeros(13, 1);
    % dxdt(1) = v1;
    % dxdt(2) = v2;
    % dxdt(3) = v3;
    % dxdt(4) = -(g*m + thrust*sin(theta)*(2*q0*q2 + 2*q1*q3) + thrust*cos(phi)*cos(theta)*(2*q2^2 + 2*q3^2 - 1) + thrust*cos(theta)*sin(phi)*(2*q0*q3 - 2*q1*q2))/m;
    % dxdt(5) = (thrust*sin(theta)*(2*q0*q1 - 2*q2*q3) - thrust*cos(theta)*sin(phi)*(2*q1^2 + 2*q3^2 - 1) + thrust*cos(phi)*cos(theta)*(2*q0*q3 + 2*q1*q2))/m;
    % dxdt(6) = (thrust*sin(theta)*(2*q1^2 + 2*q2^2 - 1) - thrust*cos(phi)*cos(theta)*(2*q0*q2 - 2*q1*q3) + thrust*cos(theta)*sin(phi)*(2*q0*q1 + 2*q2*q3))/m;
    % dxdt(7) = 0.5000*omega1*q3 - 0.5000*omega2*q2 + 0.5000*omega3*q1;
    % dxdt(8) = 0.5000*omega1*q2 - 0.5000*omega3*q0 + 0.5000*omega2*q3;
    % dxdt(9) = 0.5000*omega2*q0 - 0.5000*omega1*q1 + 0.5000*omega3*q3;
    % dxdt(10) = - 0.5000*omega1*q0 - 0.5000*omega2*q1 - 0.5000*omega3*q2;
    % dxdt(11) = ((J1_2*J2_3 - J1_3*J2_2)*(omega1*(J1_1*omega2 - J2_1*omega1) + omega2*(J1_2*omega2 - J2_2*omega1) + omega3*(J1_3*omega2 - J2_3*omega1) + rTB*thrust*cos(theta)*sin(phi)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J1_2*J3_3 - J1_3*J3_2)*(omega1*(J1_1*omega3 - J3_1*omega1) + omega2*(J1_2*omega3 - J3_2*omega1) + omega3*(J1_3*omega3 - J3_3*omega1) - rTB*thrust*sin(theta)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J2_2*J3_3 - J2_3*J3_2)*(tau + omega1*(J2_1*omega3 - J3_1*omega2) + omega2*(J2_2*omega3 - J3_2*omega2) + omega3*(J2_3*omega3 - J3_3*omega2)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1);
    % dxdt(12) = -((J1_1*J2_3 - J1_3*J2_1)*(omega1*(J1_1*omega2 - J2_1*omega1) + omega2*(J1_2*omega2 - J2_2*omega1) + omega3*(J1_3*omega2 - J2_3*omega1) + rTB*thrust*cos(theta)*sin(phi)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) - ((J1_1*J3_3 - J1_3*J3_1)*(omega1*(J1_1*omega3 - J3_1*omega1) + omega2*(J1_2*omega3 - J3_2*omega1) + omega3*(J1_3*omega3 - J3_3*omega1) - rTB*thrust*sin(theta)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) - ((J2_1*J3_3 - J2_3*J3_1)*(tau + omega1*(J2_1*omega3 - J3_1*omega2) + omega2*(J2_2*omega3 - J3_2*omega2) + omega3*(J2_3*omega3 - J3_3*omega2)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1);
    % dxdt(13) = ((J1_1*J2_2 - J1_2*J2_1)*(omega1*(J1_1*omega2 - J2_1*omega1) + omega2*(J1_2*omega2 - J2_2*omega1) + omega3*(J1_3*omega2 - J2_3*omega1) + rTB*thrust*cos(theta)*sin(phi)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J1_1*J3_2 - J1_2*J3_1)*(omega1*(J1_1*omega3 - J3_1*omega1) + omega2*(J1_2*omega3 - J3_2*omega1) + omega3*(J1_3*omega3 - J3_3*omega1) - rTB*thrust*sin(theta)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J2_1*J3_2 - J2_2*J3_1)*(tau + omega1*(J2_1*omega3 - J3_1*omega2) + omega2*(J2_2*omega3 - J3_2*omega2) + omega3*(J2_3*omega3 - J3_3*omega2)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1);

    dxdt(1) = v1;
    dxdt(2) = v2;
    dxdt(3) = v3;
    dxdt(4) = -(g*m + thrust*sin(theta)*(2*q0*q2 + 2*q1*q3) + thrust*cos(phi)*cos(theta)*(2*q2^2 + 2*q3^2 - 1) + thrust*cos(theta)*sin(phi)*(2*q0*q3 - 2*q1*q2))/m;
    dxdt(5) = (thrust*sin(theta)*(2*q0*q1 - 2*q2*q3) - thrust*cos(theta)*sin(phi)*(2*q1^2 + 2*q3^2 - 1) + thrust*cos(phi)*cos(theta)*(2*q0*q3 + 2*q1*q2))/m;
    dxdt(6) = (thrust*sin(theta)*(2*q1^2 + 2*q2^2 - 1) - thrust*cos(phi)*cos(theta)*(2*q0*q2 - 2*q1*q3) + thrust*cos(theta)*sin(phi)*(2*q0*q1 + 2*q2*q3))/m;
    dxdt(7) = 0.5000*omega1*q3 + 0.5000*omega2*q2 - 0.5000*omega3*q1;
    dxdt(8) = 0.5000*omega3*q0 - 0.5000*omega1*q2 + 0.5000*omega2*q3;
    dxdt(9) = 0.5000*omega1*q1 - 0.5000*omega2*q0 + 0.5000*omega3*q3;
    dxdt(10) = - 0.5000*omega1*q0 - 0.5000*omega2*q1 - 0.5000*omega3*q2;
    dxdt(11) = ((J1_2*J2_3 - J1_3*J2_2)*(omega1*(J1_1*omega2 - J2_1*omega1) + omega2*(J1_2*omega2 - J2_2*omega1) + omega3*(J1_3*omega2 - J2_3*omega1) + rTB*thrust*cos(theta)*sin(phi)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J1_2*J3_3 - J1_3*J3_2)*(omega1*(J1_1*omega3 - J3_1*omega1) + omega2*(J1_2*omega3 - J3_2*omega1) + omega3*(J1_3*omega3 - J3_3*omega1) - rTB*thrust*sin(theta)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J2_2*J3_3 - J2_3*J3_2)*(tau + omega1*(J2_1*omega3 - J3_1*omega2) + omega2*(J2_2*omega3 - J3_2*omega2) + omega3*(J2_3*omega3 - J3_3*omega2)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1);
    dxdt(12) = -((J1_1*J2_3 - J1_3*J2_1)*(omega1*(J1_1*omega2 - J2_1*omega1) + omega2*(J1_2*omega2 - J2_2*omega1) + omega3*(J1_3*omega2 - J2_3*omega1) + rTB*thrust*cos(theta)*sin(phi)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) - ((J1_1*J3_3 - J1_3*J3_1)*(omega1*(J1_1*omega3 - J3_1*omega1) + omega2*(J1_2*omega3 - J3_2*omega1) + omega3*(J1_3*omega3 - J3_3*omega1) - rTB*thrust*sin(theta)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) - ((J2_1*J3_3 - J2_3*J3_1)*(tau + omega1*(J2_1*omega3 - J3_1*omega2) + omega2*(J2_2*omega3 - J3_2*omega2) + omega3*(J2_3*omega3 - J3_3*omega2)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1);
    dxdt(13) = ((J1_1*J2_2 - J1_2*J2_1)*(omega1*(J1_1*omega2 - J2_1*omega1) + omega2*(J1_2*omega2 - J2_2*omega1) + omega3*(J1_3*omega2 - J2_3*omega1) + rTB*thrust*cos(theta)*sin(phi)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J1_1*J3_2 - J1_2*J3_1)*(omega1*(J1_1*omega3 - J3_1*omega1) + omega2*(J1_2*omega3 - J3_2*omega1) + omega3*(J1_3*omega3 - J3_3*omega1) - rTB*thrust*sin(theta)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1) + ((J2_1*J3_2 - J2_2*J3_1)*(tau + omega1*(J2_1*omega3 - J3_1*omega2) + omega2*(J2_2*omega3 - J3_2*omega2) + omega3*(J2_3*omega3 - J3_3*omega2)))/(J1_1*J2_2*J3_3 - J1_1*J2_3*J3_2 - J1_2*J2_1*J3_3 + J1_2*J2_3*J3_1 + J1_3*J2_1*J3_2 - J1_3*J2_2*J3_1);
end