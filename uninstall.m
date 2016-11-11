installation_path = pwd;

if ~isempty(strfind(path,pwd))
    rmpath(pwd);
    savepath;
end

cd('../');
rmdir(installation_path,'s');

disp('PCS was successfully uninstalled.');