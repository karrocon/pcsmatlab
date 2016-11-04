if isempty(strfind(path, pwd))
    addpath(pwd);
    savepath;
end

disp('PCS was successfully installed.');