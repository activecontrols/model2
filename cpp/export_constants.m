fprintf("constantsASTRA.Q << ")
for a = 1:18
    for b = 1:18
        if constantsASTRA.Q(a, b) == 0
            fprintf("0, ");
        else 
            fprintf("%.6e, ", constantsASTRA.Q(a, b));
        end 
    end
    fprintf(" //\n")
end
fprintf("\n");

fprintf("constantsASTRA.K_Att << ")
for a = 1:3
    for b = 1:9
        if constantsASTRA.K_Att(a, b) == 0
            fprintf("0, ");
        else 
            fprintf("%.6e, ", constantsASTRA.K_Att(a, b));
        end
    end
    fprintf("// \n")
end
fprintf("\n");
