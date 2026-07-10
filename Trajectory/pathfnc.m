function pos = pathfnc(fncNum, s)
    switch fncNum
        case 1
            pos = [s; 0; 3 * s];
        case 2
            r = 1;
            pos = [r*cos(2*pi*s); r*sin(2*pi*s); 3];
        otherwise
            pos = [0; 0; 0];
    end
end


