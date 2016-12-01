classdef SPGA < PCS.Control.Controller
    properties
        K    % Feedback gain matrix
        xsym % Symbolic state vector
        hsym % Symbolic control objective vector
        gsym % Symbolic vector such that \dot{x}=f+gsym*u
    end
    
    properties (Access = private)
        jac
        Lgh
        usym
        uaux
    end
    
    methods
        function self = SPGA(K,xsym,hsym,gsym)
            for i=1:1:length(xsym)
                if xsym(i)-conj(xsym(i)) ~= 0
                    error('State must be real.');
                end
            end
            if isvector(hsym) ~= 1
                error('Control objective must be a vector.');
            end
            if size(gsym) ~= [length(xsym) length(K)]
                error('Invalid control signal dimensions.');
            end
           
            self.K = K;
            self.xsym = xsym;
            self.hsym = hsym;
            self.gsym = gsym;

            self.n_inputs = length(hsym);
            self.n_outputs = length(K);
            self.n_states = 0;
            
            %The symbolic control law is computed
            self.jac = jacobian(self.hsym,self.xsym);
            self.Lgh = self.jac*self.gsym;
            self.usym = -self.K*(self.Lgh)'*self.hsym;
            self.usym = matlabFunction(self.usym,'vars',{self.xsym});
        end

        function dxcdt = derivatives(~, ~, ~, ~, ~, ~, ~)
            dxcdt = [];
        end

        function u = outputs(self, t, ~, x, ~, ~, ~)
            xt = x(t);
%             for i=1:1:length(self.xsym)
%                 self.xsym(i)=xt(i)
%             end
            %x1=xt(1); x2=xt(2); x3=xt(3); x4=xt(4);
            u=self.usym(xt);
            %u=self.usym;
            %The current values of the state are replaced in the symbolic u
            %for i=1:1:length(self.xsym)
            %   u = subs(u,self.xsym(i),xt(i));
            %end
        end
    end
end