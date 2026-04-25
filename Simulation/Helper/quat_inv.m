% Quaternion inverse
% compatible with symbolic variables

function qinv = quat_inv(q)
    qinv = q;
    qinv(2:4) = -qinv(2:4);
    qnorm2 = qinv(1)^2 + qinv(2)^2 + qinv(3)^2 + qinv(4)^2;
    qinv = qinv / qnorm2;

end