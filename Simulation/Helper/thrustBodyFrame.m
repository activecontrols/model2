% Returns the body frame thrust for a given input vector.
function TB = thrustBodyFrame(u)
    theta = u(1);
    phi = u(2);
    thrust = u(3);

    TB = thrust * [cos(theta)*sin(phi); -sin(theta); cos(theta)*cos(phi)];
end