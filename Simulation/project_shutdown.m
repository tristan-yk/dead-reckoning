projectRoot = fileparts(mfilename("fullpath"));
folders = {"Plant", "EKF"};
for i = 1:numel(folders)
    p = fullfile(projectRoot, folders{i});
    if isfolder(p)
        allPaths = strsplit(genpath(p), pathsep);
        allPaths = allPaths(~cellfun("isempty", allPaths));
        if ~isempty(allPaths)
            rmpath(allPaths{:});
        end
    end
end
