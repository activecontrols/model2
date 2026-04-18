% se23_vee
%
% Applies vee operator to xi^ in se_2(3)
%
%   (xi^)\/ = xi

function xi = se23_vee(xi_wedge)
    xi_v = xi_wedge(1:3,4);
    xi_p = xi_wedge(1:3,5);
    xi_R = [xi_wedge(3,2);
             xi_wedge(1,3);
             xi_wedge(2,1)];
    xi = [xi_v; 
          xi_p; 
          xi_R];
end