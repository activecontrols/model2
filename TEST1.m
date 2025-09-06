diff = zeros(20, 3);
c = 1;
ref = [0; 2; 0];
for i = 0:0.01:0.2   
    x = [zeros(6,1); 0.2; 0.3; i; zeros(6,1)];
    
    % Noise model for yaw (assumed known from magnetometer)
    q = x(7:9);
    q0 = sqrt(abs(1 - q(1)^2 - q(2)^2 - q(3)^2));
    
    J_h = -JacobianH(x);
    H1 = [eye(3) zeros(3, 12); 
         zeros(3,3) eye(3) zeros(3,9);  
         zeros(3,6) J_h' zeros(3,6); zeros(3,9) eye(3) zeros(3)];
    
    J_h2 = zetaCross(quatRot([q0; q]) * ref);
    H2 = [eye(3) zeros(3, 12); 
         zeros(3,3) eye(3) zeros(3,9);  
         zeros(3,6) J_h2 zeros(3,6); zeros(3,9) eye(3) zeros(3)];

    magneto = quatRot([q0; q]) * ref;
    EST = H1 * x;
    diff(c, 1:3) = magneto - (EST(7:9) + ref);
    c = c + 1;
end

[X_EST, Inn] = EstimateState2([0; 0; 0; 0; 0; 0; 0.1; 0.8; 0.1; 0; 0; 0], [zeros(6,1); [0.1;0.1;0.1]; zeros(6,1)], [0; 0; 1.5; 0], 5);



