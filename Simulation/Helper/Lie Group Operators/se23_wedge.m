% se23_wedge
% 
% Applies wedge operator to vector Lie algebra element, xi
%

function xi_wedge = se23_wedge(xi)
    xi_v = xi(1:3,:);
    xi_p = xi(4:6,:);
    xi_R= xi(7:9,:);
    
    xi_R_wedge = zetaCross(xi_R); % wedge operator in SO(3) is the cross product matrix
    xi_wedge = [xi_R_wedge, xi_v, xi_p;
                zeros(1,3), 0, 0;
                zeros(1,3), 0, 0];
end