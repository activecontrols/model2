vars = whos;  % Get info on all workspace variables
vars = vars(strcmp({vars.class}, 'double'));  % Filter only doubles

if isempty(vars)
    disp('No double variables found in workspace.');
else
    fprintf('{');
    for k = 1:length(vars)
        sz = sprintf('%dx', vars(k).size);
        sz(end) = []; % remove trailing 'x'
        fprintf('"%s": "%s",\n', vars(k).name, sz);
    end
    fprintf('}\n');
end
