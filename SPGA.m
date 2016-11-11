classdef SPGA < PCS.Control.Controller
    properties
        K    % Feedback gain matrix
    end
    
    properties (Access = private)
        xsym
        hsym
        gsym
        jac
        Lgh
        usim
    end

    methods
        function self = SPGA(K,xsim,hsim,gsim)
            for i=1:1:length(xsym)
                if xsym(i)-xsym(i)' ~= 0
                    error('State must be real.');
                end
            end
            if ndims(hsym) > 2
                error('Integral times must be a vector.');
            end
           
            self.K = K;

%           self.n_inputs = length(K);
%           self.n_outputs = length(K);
%           self.n_states = length(K);
        end

        function dxcdt = derivatives(~, ~, ~, ~, ~, ~, ~)
            dxcdt = [];
        end

        function u = outputs(self, t, xc, x, ~, ~, ~)
            if t == 0
                [self.xsym, self.hsym, self.gsym] = self.h(t, x, u, y, d);
                self.jac = jacobian(self.hsym,self.xsym);
                self.Lgh = self.jac*self.gsym;
                self.usym = -self.K*(self.Lgh)'*self.hsym;
            end
            
            for i=1:1:length(self.xsim)
                self.usim = subs(self.usim,self.xim(i),x(i));
            end
            u = self.usim;
        end
    end
    
    methods (Abstract)
        [xsym,hsym,usym] = uCompt(self, t, x, u, y, d);
    end
end