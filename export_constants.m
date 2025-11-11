fprintf("constantsASTRA.Q << ")
for i = 1:1:143
    if constantsASTRA.Q(i) == 0
        fprintf("0, ");
    else 
        fprintf("%.6e, ", constantsASTRA.Q(i));
    end
end

if constantsASTRA.Q(144) == 0
    fprintf("0;\n");
else 
    fprintf("%.6e;\n", constantsASTRA.Q(144));
end

fprintf("K << ")
for i = 1:1:47
    if K(i) == 0
        fprintf("0, ");
    else 
        fprintf("%.6e, ", K(i));
    end
end

if K(48) == 0
    fprintf("0;\n");
else 
    fprintf("%.6e;\n", K(48));
end
