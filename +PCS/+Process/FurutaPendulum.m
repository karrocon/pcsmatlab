classdef FurutaPendulum < PCS.Process.Process
    properties
        % Mechanical parameters of the system
        m  % mass [rotary arm   pendulum]
        L  % total length [rotary arm   pendulum]
        l  % Length to the center of mass [rotary arm   pendulum]
        J  % Momentum of inertia [rotary arm   pendulum]
        B  % Viscous damping coefficient [rotary arm   pendulum]
        g  % Acceleration of gravity
    end

    methods
        function self = FurutaPendulum(m, L, l, J, B, g)
            if isvector(m) ~= 1 || length(m) ~= 2
                error('Mass must be a vector of length 2 (arm and pendulum).');
            end
            if isvector(L) ~= 1 || length(L) ~= 2
                error('Total length must be a vector of length 2 (arm and pendulum).');
            end
            if isvector(l) ~= 1 || length(l) ~= 2
                error('Length to the center of mass must be a vector of length 2 (arm and pendulum).');
            end
            if isvector(J) ~= 1 || length(J) ~= 2
                error('Momentum of inertia must be a vector of length 2 (arm and pendulum).');
            end
            if isvector(B) ~= 1 || length(B) ~= 2
                error('Viscous damping coefficient must be a vector of length 2 (arm and pendulum).');
            end
            if isscalar(g) ~= 1
                error('Acceleration of gravity must be a scalar.');
            end

            self.m = m;
            self.L = L;
            self.l = l;
            self.J = J;
            self.B = B;
            self.g = g;

            self.n_inputs = 1;
            self.n_outputs = 4;
            self.n_states = 4;
        end

        function dxdt = derivatives(self, t, x, u, ~, ~)
            dxdt = zeros(4, 1);
            xt = x(t);
            ut = u(t);

            M11=self.J(1)+self.m(2)*(self.L(1)^2+self.l(2)^2*sin(xt(2))^2);
            M12=self.l(2)*self.m(2)*self.L(1)*cos(xt(2));
            M22=self.J(2)+self.l(2)^2*self.m(2);
            M=[M11 M12; M12 M22];

            C11=2*(self.m(2)*self.l(2)^2*sin(xt(2))*cos(xt(2))*xt(4));
            C12=-xt(4)*sin(xt(2))*self.L(1)*self.l(2)*self.m(2);
            C21=-xt(3)*cos(xt(2))*self.l(2)^2*self.m(2)*sin(xt(2));
            C=[C11 C12; C21 0];

            G2=-self.g*self.l(2)*self.m(2)*sin(xt(2));
            G=[0;G2];

            U=[1;0];
            
            dxdt=[xt(3);xt(4);inv(M)*(U*ut-C*xt(3:4)-G)];
            
        end

        function y = outputs(self, t, x, ~, ~, ~)
            y = x(t);
        end
    end
end