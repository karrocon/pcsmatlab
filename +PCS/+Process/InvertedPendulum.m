classdef InvertedPendulum < PCS.Process.Process
    properties
        % Mechanical parameters of the system
        m  % Mass
        L  % Total length
        l  % Length to the center of mass
        J  % Momentum of inertia
        B  % Viscous damping coefficient
        g  % Acceleration of gravity
    end

    methods
        function self = InvertedPendulum(m, L, l, J, B, g)
            if isscalar(m) ~= 1
                error('Mass must be scalar.');
            end
            if isscalar(L) ~= 1
                error('Total length must be scalar.');
            end
            if isscalar(l) ~= 1
                error('Length to the center of mass must be scalar.');
            end
            if isscalar(J) ~= 1
                error('Momentum of inertia must be scalar.');
            end
            if isscalar(B) ~= 1
                error('Viscous damping coefficient must be scalar.');
            end
            if isscalar(g) ~= 1
                error('Acceleration of gravity must be scalar.');
            end

            self.m = m;
            self.L = L;
            self.l = l;
            self.J = J;
            self.B = B;
            self.g = g;

            self.n_inputs = 1;
            self.n_outputs = 2;
            self.n_states = 2;
        end

        function dxdt = derivatives(self, t, x, u, ~, ~)
            dxdt = zeros(2, 1);
            xt = x(t);
            ut = u(t);

            dxdt(1) = xt(2);
            dxdt(2) = self.m*self.g*self.l*sin(xt(1))/self.J-self.m*self.l*cos(xt(1))*ut/self.J;
        end

        function y = outputs(self, t, x, ~, ~, ~)
            y = x(t);
        end
    end
end