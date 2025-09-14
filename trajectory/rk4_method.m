function x = rk4_method(odefun, k_list, dt, x0, U)
    x = zeros(length(x0), length(k_list));
    x(:, 1) = x0;

    for k = k_list
        u = U(:, k);

        x_k1 = x(:, k);
        K1 = dt * odefun(x_k1(1), x_k1(2), x_k1(3), x_k1(4), x_k1(5), x_k1(6), x_k1(7), x_k1(8), x_k1(9), x_k1(10), x_k1(11), x_k1(12), x_k1(13), u(1), u(2), u(3), u(4));
        
        x_k2 = x(:, k) + K1/2;
        K2 = dt * odefun(x_k2(1), x_k2(2), x_k2(3), x_k2(4), x_k2(5), x_k2(6), x_k2(7), x_k2(8), x_k2(9), x_k2(10), x_k2(11), x_k2(12), x_k2(13), u(1), u(2), u(3), u(4));
        
        x_k3 = x(:, k) + K2/2;
        K3 = dt * odefun(x_k3(1), x_k3(2), x_k3(3), x_k3(4), x_k3(5), x_k3(6), x_k3(7), x_k3(8), x_k3(9), x_k3(10), x_k3(11), x_k3(12), x_k3(13), u(1), u(2), u(3), u(4));
        
        x_k4 = x(:, k) + K3;
        K4 = dt * odefun(x_k4(1), x_k4(2), x_k4(3), x_k4(4), x_k4(5), x_k4(6), x_k4(7), x_k4(8), x_k4(9), x_k4(10), x_k4(11), x_k4(12), x_k4(13), u(1), u(2), u(3), u(4)); 
    
        x(:, k+1) = x(:, k) + K1 / 6 + K2 / 3 + K3 / 3 + K4 / 6;
    end
end