fprintf("constantsASTRA.Q << ")
for a = 1:12
    for b = 1:12
        if constantsASTRA.Q(a, b) == 0
            fprintf("0, ");
        else 
            fprintf("%.6e, ", constantsASTRA.Q(a, b));
        end
    end
end
fprintf("\n");

fprintf("K << ")
for a = 1:4
    for b = 1:12
        if K(a, b) == 0
            fprintf("0, ");
        else 
            fprintf("%.6e, ", K(a, b));
        end
    end
end
fprintf("\n");
