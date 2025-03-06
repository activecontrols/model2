
function constants = constructConstants
    data = readcell('constants.xlsx');%, 'ReadVariableNames', false);
    constants = struct();
    
    for i = 2:height(data)
        varName = data{i,1};
        value = data{i,2};
    
        if ischar(value) && (contains(value, ';') || contains(value, ','))  % Example check: semicolon indicates a matrix-like input
            % Convert the string representation of the matrix into an actual matrix
            value = str2num(value);  % Assuming the matrix is provided as a string, like '1 2; 3 4'
        end

        constants.(varName) = value;
    end
end