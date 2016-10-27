classdef Utils
    %Utils Static utilities library
    
    methods (Static = true)
        function y = lagrange_interpolation(X,Y,x)
            % Compute Lagrange interpolation
            n = length(X); 
            Lj = zeros(n);
            
            jr = 1:n; 
            for j = jr 
               multiplier = 1;
               outputConv = 1; 
               mr = jr(jr ~= j); 
               for m = mr
                   outputConv = conv(outputConv,[1 -X(m)]);
                   multiplier = multiplier * ((X(j) - X(m))^-1);
               end
               Lj(j,:) = multiplier * outputConv; 
            end
            
            L = Y * Lj;
            
            y = polyval(L,x);
        end
        
        function classes = subclasses(class,folder,depth)
            % Get MATLAB subclasses for a given class
            if nargin == 1
                folder = '.';
                depth = 0;
            elseif nargin == 2
                depth = 0;
            end
            
            if ~exist(folder,'dir')
                error ('Error: The provided folder does not exist.');
            end
            
            classes = CSS.Utils.Utils.subclasses_internal(class,folder,depth);
        end
    end
    
    methods (Static = true, Access = private)
        function classes = subclasses_internal(class,folder,depth)
            % Internal recursive method for obtaning MATLAB subclasses
            classes = cell(0);
            
            files = dir(folder);
            for i=1:length(files)
                if ~strcmp(files(i).name,'.') && ~strcmp(files(i).name,'..')
                    if files(i).isdir && depth > 0
                        classes_aux = CSS.Utils.Utils.subclasses_internal(class,strcat(folder,'/',files(i).name),depth-1);
                        
                        for j=1:length(classes_aux)
                            if ~isempty(classes_aux{j})
                                classes{end+1} = classes_aux{j};
                            end
                        end
                    else
                        file_splitted = strsplit(files(i).name,'.');
                        if length(file_splitted) == 2
                            file_name = file_splitted{1};
                            file_extension = file_splitted{2};
                            if strcmp(file_extension,'m')
                                if any(folder == '+')
                                    folder_splitted = strsplit(folder,'+');
                                    for j=length(folder_splitted):-1:2
                                        file_name = strcat(strrep(folder_splitted{j},'/',''),'.',file_name);
                                    end
                                end
                                if exist(file_name,'class') && ismember(class, superclasses(file_name))
                                    classes{end+1} = file_name;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end