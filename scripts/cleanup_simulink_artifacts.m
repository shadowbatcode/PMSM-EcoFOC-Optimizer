function cleanup_simulink_artifacts(p)
%CLEANUP_SIMULINK_ARTIFACTS Remove generated Simulink cache artifacts.

if nargin < 1 || ~isstruct(p) || ~isfield(p, 'project_root')
    project_root = fileparts(fileparts(mfilename('fullpath')));
else
    project_root = p.project_root;
end

paths_to_delete = {
    fullfile(project_root, 'slprj')
    fullfile(project_root, 'scripts', 'slprj')
};

for i = 1:numel(paths_to_delete)
    path_i = paths_to_delete{i};
    if exist(path_i, 'dir') == 7
        try
            rmdir(path_i, 's');
        catch
        end
    end
end

slxc_files = [
    dir(fullfile(project_root, '*.slxc'))
    dir(fullfile(project_root, 'scripts', '*.slxc'))
];

for i = 1:numel(slxc_files)
    try
        delete(fullfile(slxc_files(i).folder, slxc_files(i).name));
    catch
    end
end

% Keep the canonical tree clean after model generation and simulation.
end
