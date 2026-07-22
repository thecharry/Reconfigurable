% MATLAB startup configuration for RC-Designer and App Designer.
% Keep CORA/GPTIPS available while preventing its extract.m from shadowing
% MATLAB's own extract function.
gptipsPath = '/Users/charry/CORA/global/thirdparty/gptips2';
if isfolder(gptipsPath) && contains([path, pathsep], [gptipsPath, pathsep])
    rmpath(gptipsPath);
    addpath(gptipsPath, '-end');
end
clear gptipsPath
rehash;
