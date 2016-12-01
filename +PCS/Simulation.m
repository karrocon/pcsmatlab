classdef Simulation < handle
    %Simulation
    
    properties
        process
        controller
    end
    
    %%%%%%%%%%%
    % PRIVATE %
    %%%%%%%%%%%
    properties
        % Simulation options
        solver = @ode45
        t0
        tend
        
        % Initial states
        d0
        r0
        u0
        x0
        xc0
        y0
        
        % Exogenous fun
        d_preloaded_fun
        r_preloaded_fun
    end
    
    properties
        t_events_delay
        
        % Matrix format: [index time]
        t_sim
        d_sim % Matrix containing real disturbances values
        d_measured_sim % Matrix containing measured disturbances values
        d_sent_sim
        r_sim
        u_sim
        u_actuator_sim
        u_sent_sim
        xc_sim
        xp_sim
        xp_measured_sim
        xp_sent_sim
        y_sim
        y_measured_sim
        y_sent_sim
        te_sim
    end
    
    % Sensors
    properties (Hidden = true)
        actuators
        
        x_sensors
        d_sensors
        y_sensors
        
        sensor_links
        actuator_links
    end
    
    properties (Dependent = true)
        % Simulation options
        abs_tol = 1e-6
        bdf = 'off'
        initial_step = 0
        max_order = 5
        max_step = 0
        non_negative = 0
        refine = 1
        rel_tol = 1e-3
    end
    
    properties
        options
        
        show_waitbar = 1
        use_delay_events = 0
        
        event_tol = 1e-6
        
        verbose = 0
        
        zeno_max_deep = 2
        on_zeno_enter
        on_zeno_exit
    end
    
    properties (Access = private, Hidden = true)
        ct_actuator_links
        dt_actuator_links
        ce_actuator_links
        de_actuator_links
        
        ct_sensor_links
        dt_sensor_links
        ce_sensor_links
        de_sensor_links
    end
    
    properties (Access = private)
        % Adaptive solver options
        adaptive_initial_step
        adaptive_max_step
        
        % Function signals
        
        
        % Waitbar
        h_wb
    end
    
    %%%%%%%%%%%%%%%
    % CONSTRUCTOR %
    %%%%%%%%%%%%%%%
    methods
        function self = Simulation(controller,process)
            %% Simulation  Construct a simulation object.
            %    Simulation(CONTROLLER, PROCESS) constructs a simulation
            %    object for given CONTROLLER and PROCESS system objects 
            %    (see help system).
            self.t0 = 0;
            self.tend = 1;
            
            if nargin == 0
                self.sensor_links{1} = PCS.Network.SensorLink();
                self.actuator_links{1} = PCS.Network.ActuatorLink();
            else
                if process.n_inputs ~= controller.n_outputs
                    error('Process inputs are different to controller outputs');
                end

                self.controller = controller;
                self.process = process;
                
                % Initial states
                self.d0 = zeros(process.n_disturbances,1);
                self.r0 = zeros(process.n_outputs,1);
                self.u0 = zeros(process.n_inputs,1);

                self.x0 = zeros(process.n_states,1);
                self.xc0 = zeros(controller.n_states,1);

                self.y0 = zeros(process.n_outputs,1);
                
                % Preloaded functions
                for i = 1:process.n_disturbances
                    self.set_preloaded_disturbance(i,0);
                end
                
                for i = 1:process.n_outputs
                    self.set_preloaded_reference(i,0);
                end

                % Default perfect sensors
                for i = 1:process.n_disturbances
                    self.d_sensors{i} = PCS.Hardware.Sensor();
                end

                for i = 1:process.n_states
                    self.x_sensors{i} = PCS.Hardware.Sensor();
                end

                for i = 1:process.n_outputs
                    self.y_sensors{i} = PCS.Hardware.Sensor();
                end

                % Default perfect actuators
                for i = 1:process.n_inputs
                    self.actuators{i} = PCS.Hardware.Actuator();
                end
                
                % Links
                self.sensor_links{1} = PCS.Network.SensorLink();
                self.actuator_links{1} = PCS.Network.ActuatorLink();

                self.sensor_links{1}.disturbances_to_transmit = 1:self.process.n_disturbances;
                self.sensor_links{1}.states_to_transmit = 1:self.process.n_states;
                self.sensor_links{1}.outputs_to_transmit = 1:self.process.n_outputs;

                self.actuator_links{1}.control_signals_to_transmit = 1:self.process.n_inputs;
            end
            
            % Initialize ode solver options
            self.options = odeset;
            
            self.options.Events = @self.events;
            self.options.OutputFcn = @self.outputs;
        end
    end
    
    %%%%%%%%%%%
    % GET/SET %
    %%%%%%%%%%%
    methods
        % Simulation options
        function abs_tol = get.abs_tol(self)
            abs_tol = self.options.AbsTol;
        end
        
        function set.abs_tol(self,abs_tol)
            if abs_tol < 0
                error ('Error: Absolute error tolerance must a positive scalar or vector.');
            end
            
            self.options.AbsTol = abs_tol;
        end
        
        function bdf = get.bdf(self)
            bdf = self.options.BDF;
        end
        
        function set.bdf(self,bdf)
            if ~strcmp(bdf,'on') && ~strcmp(bdf,'off')
                error ('Error: Backward Differentiation Formulas property must be [''on'',''off''].');
            end
            
            if ~isequal(self.solver,@ode15s)
                warning ('Warning: Backward Differentiation Formulas are only available for ODE15S by default.')
            end
            
            self.options.BDF = bdf;
        end
        
        function initial_step = get.initial_step(self)
            initial_step = self.options.InitialStep;
        end
        
        function set.initial_step(self,initial_step)
            if initial_step < 0
                error ('Error: Initial step must be a positive scalar for constant value or 0 por adaptive.');
            end
            
            if initial_step > self.tend - self.t0
                warning ('Warning: Initial step is greater than simulation length.');
            end
            
            self.options.InitialStep = initial_step;
        end
        
        function max_order = get.max_order(self)
            max_order = self.options.MaxOrder;
        end
        
        function set.max_order(self,max_order)
            if max_order < 1 || max_order > 5
                error ('Error: Maximum order option must be an integer in the range [1,5].');
            end
            
            if ~isequal(self.solver,@ode15s) && ~isequal(self.solver,@ode15i)
                warning ('Warning: Maximum order option is only available for ODE15S and ODE15I by default.');
            end
            
            self.options.MaxOrder = max_order;
        end
        
        function max_step = get.max_step(self)
            max_step = self.options.MaxStep;
        end
        
        function set.max_step(self,max_step)
            if max_step < 0
                error ('Error: Maximum step must be a positive scalar or 0 for adaptive.');
            end
            
            if max_step > self.tend-self.t0
                warning ('Warning: Maximum step is greater than simulation length.');
            end
            
            self.options.MaxStep = max_step;
        end
        
        function non_negative = get.non_negative(self)
            non_negative = self.options.NonNegative;
        end
        
        function set.non_negative(self,non_negative)
            self.options.NonNegative = non_negative;
        end
        
        function refine = get.refine(self)
            refine = self.options.Refine;
        end
        
        function set.refine(self,refine)
            if ~isscalar(refine)
                error ('Error: Refine must be a positive integer.');
            end
            
            if round(refine) ~= refine || refine <= 0
                error ('Error: Refine must be a positive integer.');
            end
            
            self.options.Refine = refine;
        end
        
        function rel_tol = get.rel_tol(self)
            rel_tol = self.options.RelTol;
        end
        
        function set.rel_tol(self,rel_tol)
            if rel_tol < 0
                error ('Error: Relative error tolerance property must be a positive scalar.');
            end
            
            self.options.RelTol = rel_tol;
        end
        
        function solver = get.solver(self)
            solver = self.solver;
        end
        
        function set.solver(self,solver)
            if ~exist(func2str(solver),'file') && ~exist(func2str(solver),'builtin')
                error ('Error: Solver function does not exist.');
            end
            
            self.solver = solver;
        end
        
        function t0 = get.t0(self)
            t0 = self.t0;
        end
        
        function set.t0(self,t0)
            if length(t0) > 1 || isempty(t0) || iscell(t0)
                error ('Error: Initial time must be an scalar.');
            end
            
            if t0 < 0
                error ('Error: Initial time must be positive or zero.');
            elseif t0 >= self.tend
                error ('Error: Initial time must be smaller than final time.');
            end
            
            self.t0 = t0;
        end
        
        function tend = get.tend(self)
            %%
            % *tend* Get final time.
            tend = self.tend;
        end
        
        function set.tend(self,tend)
            if tend < 0
                error ('Error: Final time must be positive or zero.');
            elseif tend <= self.t0
                error ('Error: Final time must be greater than initial time.');
            end
            
            self.tend = tend;
        end
        
        % Initial states
        function d0 = get.d0(self)
            d0 = self.d0;
        end
        
        function set.d0(self,d0)
            self.d0 = d0;
        end
        
        function r0 = get.r0(self)
            r0 = self.r0;
        end
        
        function set.r0(self,r0)
            self.r0 = r0;
        end
        
        function u0 = get.u0(self)
            u0 = self.u0;
        end
        
        function set.u0(self,u0)
            if self.process.n_inputs ~= length(u0)
                error ('Error: The length of u0 does not match the number of inputs of the process.');
            end
            
            self.u0 = u0;
        end
        
        function x0 = get.x0(self)
            x0 = self.x0;
        end
        
        function set.x0(self,x0)
            if self.process.n_states ~= length(x0)
                error ('Error: The length of x0 does not match the number of states of the process.');
            end
            
            self.x0 = x0;
        end
        
        function xc0 = get.xc0(self)
            xc0 = self.xc0;
        end
        
        function set.xc0(self,xc0)
            if self.controller.n_states ~= length(xc0)
                error ('Error: The length of xc0 does not match the number of states of the controller.');
            end
            
            self.xc0 = xc0;
        end
        
        function y0 = get.y0(self)
            y0 = self.y0;
        end
        
        function set.y0(self,y0)
            if self.process.n_outputs ~= length(y0)
                error ('Error: The length of y0 does not match the number of outputs of the process.');
            end
            
            self.y0 = y0;
        end
        
        % Controller/Process
        function set.controller(self,controller)
            if ~isscalar(controller) || ~isa(controller,'PCS.Control.Controller')
                error ('Error: Controller must be a subclass of PCS.Control.Controller.');
            end
            
            for i=1:controller.n_outputs
                self.actuators{i} = PCS.Hardware.Actuator();
            end
            
            self.controller = controller;
        end
        
        function set.process(self,process)
            if ~isscalar(process) || ~isa(process,'PCS.Process.Process')
                error ('Error: Process must be a subclass of PCS.Process.Process.');
            end
            
            for i=1:process.n_disturbances
                self.d_sensors{i} = PCS.Hardware.Sensor();
            end
            
            for i=1:process.n_outputs
                self.y_sensors{i} = PCS.Hardware.Sensor();
            end
            
            for i=1:process.n_states
                self.x_sensors{i} = PCS.Hardware.Sensor();
            end
            
            self.sensor_links = [];
            self.add_sensor_link(PCS.Network.SensorLink());
            self.sensor_links{1}.disturbances_to_transmit = 1:process.n_disturbances;
            self.sensor_links{1}.states_to_transmit = 1:process.n_states;
            self.sensor_links{1}.outputs_to_transmit = 1:process.n_outputs;
            
            self.process = process;
        end
        
        % Sensors
        function set.x_sensors(self,sensor)
            if iscell(sensor)
                for i=1:length(sensor)
                    if ~isa(sensor{i},'PCS.Hardware.Sensor')
                        error ('Error: A Sensor object was expected.');
                    end
                end
                
                self.x_sensors = sensor;
            elseif ~isa(sensor,'PCS.Hardware.Sensor')
                error ('Error: A Sensor object was expected.');
            end
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%
    % OPERATOR OVERLOADS %
    %%%%%%%%%%%%%%%%%%%%%%
    methods
        function S = check_subs(self,S)
            field_name = S(1).subs;
            
            % Primitive types
            if strcmp(field_name,'max_step') || strcmp(field_name,'solver') ...
                || strcmp(field_name,'t0')
                if length(S) > 1
                    error ('Error: Wrong use of %s property.',field_name);
                end
            end
            
            % Non-primitive types
            
            % sensors
            if length(S) >= 2
                if strcmp(field_name,'actuators')
                    for index = S(2).subs{1}
                        if index <= 0 || index > self.process.n_inputs
                            error ('Error: The index must be an integer between 1 and the number of inputs.');
                        end
                    end

                    S(2).type = '{}';
                end
                
                if strcmp(field_name,'d_sensors')
                    for index = S(2).subs{1}
                        if index <= 0 || index > self.process.n_states
                            error ('Error: The index must be an integer between 1 and the number of measurable disturbances.');
                        end
                    end

                    S(2).type = '{}';
                end

                if strcmp(field_name,'x_sensors')
                    for index = S(2).subs{1}
                        if index <= 0 || index > self.process.n_states
                            error ('Error: The index must be an integer between 1 and the number of states of the process.');
                        end
                    end

                    S(2).type = '{}';
                end

                if strcmp(field_name,'y_sensors')
                    for index = S(2).subs{1}
                        if index <= 0 || index > self.process.n_outputs
                            error ('Error: The index must be an integer between 1 and the number of outputs of the process.');
                        end
                    end

                    S(2).type = '{}';
                end

                % sensors links
                if strcmp(field_name,'sensor_links')
                    for index = S(2).subs{1}
                        if index <= 0 || index > length(self.sensor_links)
                            error ('Error: The index must be an integer between 1 and the number of sensor links.');
                        end
                    end

                    S(2).type = '{}';
                end
            end
        end
        
        function varargout = subsasgn(self,S,b)
            % Overloaded subasgn
            S = self.check_subs(S);
            
            [varargout{1:nargout}] = builtin('subsasgn',self,S,b);
        end
        
        function varargout = subsref(self,S)
            % Overloaded subsref
            S = self.check_subs(S);
            
            [varargout{1:nargout}] = builtin('subsref',self,S);
        end
    end
    
    %%%%%%%%%%
    % PUBLIC %
    %%%%%%%%%%
    methods
        function add_actuator_link(self,link)
            self.actuator_links{end+1} = link;
        end
        
        function add_sensor_link(self,link)
            self.sensor_links{end+1} = link;
        end
        
        function set_preloaded_disturbance(self,index,d,t,interp)
            %%TODO Move to its proper place
            if ~isscalar(index) || ~isnumeric(index) || index <= 0 || index > self.process.n_disturbances
                error('Error: Index must be an integer between 1 and the number of disturbances.');
            end
            
            if nargin == 3
                self.d_preloaded_fun{index} = @(varargin) d;
            elseif nargin == 4
                self.d_preloaded_fun{index} = self.create_signal_fun(t,d,PCS.Utils.InterpolationMethod.ZOH);
            else
                self.d_preloaded_fun{index} = self.create_signal_fun(t,d,interp);
            end
        end
        
        function set_preloaded_reference(self,index,r,t,interp)
            if nargin == 3
                self.r_preloaded_fun{index} = @(varargin) r;
            elseif nargin == 4
                self.r_preloaded_fun{index} = self.create_signal_fun(t,r,PCS.Utils.InterpolationMethod.ZOH);
            else
                self.r_preloaded_fun{index} = self.create_signal_fun(t,r,interp);
            end
        end
        
        function data = run(self,t_vector)
            self.check_integrity();
            
            if nargin == 1
                t_vector = [];
            end
            
            self.init_simulation();
            
            while self.t_sim(end) < self.tend
                if self.controller.n_states == 0
                    [t,x,te,xe,ie] = self.solver(@(t,x)self.derivatives(t,x),[self.t_sim(end) self.tend],self.xp_sim(:,end),self.options);
                else
                    [t,x,te,xe,ie] = self.solver(@(t,x)self.derivatives(t,x),[self.t_sim(end) self.tend],[self.xp_sim(:,end);self.xc_sim(:,end)],self.options);
                end
                
                if self.show_waitbar
                    if getappdata(self.h_wb,'canceling')
                        disp('Simulation cancelled by user.');
                        
                        data = self.end_simulation(t_vector);
                        
                        return;
                    end
                end
                
                % Handle events
                if ~isempty(ie)
                    status = self.on_event(ie,te);
                    
                    if status == 1
                        error ('Error: Zeno effect could not be resolved.');
                    elseif status == -1
                        error('Error: Unhandled zeno exception. Define on_zeno_enter function to handle zeno effect.');
                    end
                    
                    if self.adaptive_initial_step
                        self.initial_step = self.t_sim(end) - self.t_sim(end-self.refine);
                    end
                    
                    if self.adaptive_max_step
                        self.max_step = self.tend-self.t_sim(end);
                        %self.max_step = self.t_sim(end)-t(1);
                    end
                end
            end

            data = self.end_simulation(t_vector);
            
            if nargout == 0
                self.plot();
            end
        end
    end
    
    %%%%%%%%%%%
    % PRIVATE %
    %%%%%%%%%%%
    methods (Hidden = true, Access = private)
        function [dxdt,d,r,u,y,xp_measured,y_measured,d_measured,xp_sent,y_sent,d_sent,u_sent,u_actuator] = derivatives(self,t,x)
            % Load preloaded signals
            d = zeros(self.process.n_disturbances,1);
            for i = 1:self.process.n_disturbances
                d(i) = self.d_preloaded_fun{i}(t);
            end
            
            d_measured = d;
            for i = 1:length(d)
                if isempty(self.d_sensors{i})
                    % No sensor available
                    d_measured(i) = NaN;
                else
                    % Sensor available
                    d_measured(i) = self.d_sensors{i}.read(d(i));
                end
            end
            
            if isempty(self.d_measured_sim)
                self.d_measured_sim = d_measured;
            end
            
            d_fun = self.create_signal_fun([self.t_sim t],[self.d_sim d],PCS.Utils.InterpolationMethod.ZOH);
            
            d_measured_fun = self.create_signal_fun([self.t_sim t],[self.d_measured_sim d_measured],PCS.Utils.InterpolationMethod.ZOH);
            
            r = zeros(self.process.n_outputs,1);
            for i = 1:self.process.n_outputs
                r(i) = self.r_preloaded_fun{i}(t);
            end
            
            r_fun = self.create_signal_fun([self.t_sim t],[self.r_sim r],PCS.Utils.InterpolationMethod.ZOH);
            
            xp = x(1:self.process.n_states);
            
            xp_measured = xp;
            for i = 1:length(xp)
                if isempty(self.x_sensors{i})
                    % No sensor available
                    xp_measured(i) = NaN;
                else
                    % Sensor available
                    xp_measured(i) = self.x_sensors{i}.read(xp(i));
                end
            end
            
            if isempty(self.xp_measured_sim)
                self.xp_measured_sim = xp_measured;
            end
            
            xp_fun = self.create_signal_fun([self.t_sim t],[self.xp_sim xp],PCS.Utils.InterpolationMethod.Linear);
            
            xp_measured_fun = self.create_signal_fun([self.t_sim t],[self.xp_measured_sim xp_measured],PCS.Utils.InterpolationMethod.ZOH);
            
            xc = x(self.process.n_states+1:end);
            
            xc_fun = self.create_signal_fun([self.t_sim t],[self.xc_sim xc],PCS.Utils.InterpolationMethod.Linear);
            
            u_actuator_fun = self.create_signal_fun(self.t_sim,self.u_actuator_sim,PCS.Utils.InterpolationMethod.ZOH);
            
            y = self.process.outputs(t,xp_fun,u_actuator_fun,d_fun);
            
            y_measured = y;
            for i = 1:length(y)
                if isempty(self.y_sensors{i})
                    % No sensor available
                    y_measured(i) = NaN;
                else
                    % Sensor available
                    y_measured(i) = self.y_sensors{i}.read(y(i));
                end
            end
            
            if isempty(self.y_measured_sim)
                self.y_measured_sim = y_measured;
            end
            
            y_measured_fun = self.create_signal_fun([self.t_sim t],[self.y_measured_sim y_measured],PCS.Utils.InterpolationMethod.ZOH);
            
            if isempty(self.xp_sent_sim)
                xp_sent = xp_measured;
                self.xp_sent_sim = xp_sent;
            else
                xp_sent = self.xp_sent_sim(:,end);
            end
            
            if isempty(self.y_sent_sim)
                y_sent = y_measured;
                self.y_sent_sim = y_sent;
            else
                y_sent = self.y_sent_sim(:,end);
            end
            
            if self.process.n_disturbances > 0
                d_sent = self.d_sent_sim(:,end);
            else
                d_sent = 0;
            end
            
            for i = 1:length(self.sensor_links)
                if self.sensor_links{i}.type == PCS.Network.LinkType.ContinuousTime
                    [xp_sent_aux,y_sent_aux,d_sent_aux] = self.sensor_links{i}.send(t,xp_measured_fun,y_measured_fun,d_measured_fun);
                    
                    index_aux = ~isnan(xp_sent_aux);
                    xp_sent(index_aux) = xp_sent_aux(index_aux);
                    
                    index_aux = ~isnan(y_sent_aux);
                    y_sent(index_aux) = y_sent_aux(index_aux);
                    
                    index_aux = ~isnan(d_sent_aux);
                    d_sent(index_aux) = d_sent_aux(index_aux);
                end
            end
            
            xp_sent_fun = self.create_signal_fun([self.t_sim t],[self.xp_sent_sim xp_sent],PCS.Utils.InterpolationMethod.ZOH);
            y_sent_fun = self.create_signal_fun([self.t_sim t],[self.y_sent_sim y_sent],PCS.Utils.InterpolationMethod.ZOH);
            d_sent_fun = self.create_signal_fun([self.t_sim t],[self.d_sent_sim d_sent],PCS.Utils.InterpolationMethod.ZOH);
            
            u = self.controller.outputs(t,xc_fun,xp_sent_fun,y_sent_fun,d_sent_fun,r_fun);
            
            u_fun = self.create_signal_fun([self.t_sim t],[self.u_sim u],PCS.Utils.InterpolationMethod.Linear);
            
            if isempty(self.u_sent_sim)
                u_sent = u;
                self.u_sent_sim = u_sent;
            else
                u_sent = self.u_sent_sim(:,end);
            end
            
            for i = 1:length(self.actuator_links)
                if self.actuator_links{i}.type == PCS.Network.LinkType.ContinuousTime
                    u_sent_aux = self.actuator_links{i}.send(t,u_fun);
                    
                    index_aux = ~isnan(u_sent_aux);
                    u_sent(index_aux) = u_sent_aux(index_aux);
                end
            end
            
            %u_sent_fun = self.create_signal_fun([self.t_sim t],[self.u_sent_sim u_sent],PCS.Utils.InterpolationMethod.ZOH);
            
            u_actuator = u_sent;
            for i = 1:length(self.actuators)
                u_actuator(i) = self.actuators{i}.write(u_sent(i));
            end
            
            if isempty(self.u_actuator_sim)
                self.u_actuator_sim = u_actuator;
            end
            
            u_actuator_fun = self.create_signal_fun([self.t_sim t],[self.u_actuator_sim u_actuator],PCS.Utils.InterpolationMethod.ZOH);
            
            dxdt = [self.process.derivatives(t,xp_fun,u_actuator_fun,d_fun); self.controller.derivatives(t,xc_fun,xp_sent_fun,y_sent_fun,d_measured_fun,r_fun)];
        end
        
        function [value,isterminal,direction] = events(self,t,x)
            delta_u = self.process.input_delay(t);
            delta_u = round(delta_u*1000)/1000;
            
            % ContinuousEventTriggered conditions
            [~,~,~,u,~,x_measured,y_measured,d_measured,~,~,~,~,~] = self.derivatives(t,x);
            
            x_measured_fun = self.create_signal_fun([self.t_sim t],[self.xp_measured_sim x_measured],PCS.Utils.InterpolationMethod.ZOH);
            y_measured_fun = self.create_signal_fun([self.t_sim t],[self.y_measured_sim y_measured],PCS.Utils.InterpolationMethod.ZOH);
            d_measured_fun = self.create_signal_fun([self.t_sim t],[self.d_measured_sim d_measured],PCS.Utils.InterpolationMethod.ZOH);
            
            u_fun = self.create_signal_fun([self.t_sim t],[self.u_sim u],PCS.Utils.InterpolationMethod.Linear);
            
            ce_conditions = zeros(length(self.ce_sensor_links)+length(self.ce_actuator_links),1);
            for i = 1:length(self.ce_sensor_links)
                ce_conditions(i) = self.sensor_links{self.ce_sensor_links(i)}.triggering_condition(t,self.sensor_links{self.ce_sensor_links(i)}.last_triggering_time,0,x_measured_fun,y_measured_fun,d_measured_fun,0,u_fun);
            end
            for i = length(self.ce_sensor_links)+1:length(ce_conditions)
                ce_conditions(i) = self.actuator_links{self.ce_actuator_links(i-length(self.ce_sensor_links))}.triggering_condition(t,self.actuator_links{self.ce_actuator_links(i-length(self.ce_sensor_links))}.last_triggering_time,0,x_measured_fun,y_measured_fun,d_measured_fun,0,u_fun);
            end
            
            % DiscreteTime conditions
            dt_conditions = zeros(length(self.dt_sensor_links)+length(self.dt_actuator_links),1);
            for i = 1:length(self.dt_sensor_links)
                dt_conditions(i) = self.sensor_links{self.dt_sensor_links(i)}.last_sampling_time+self.sensor_links{self.dt_sensor_links(i)}.sampling_time-t;
            end
            for i = length(self.dt_sensor_links)+1:length(dt_conditions)
                dt_conditions(i) = self.actuator_links{self.dt_actuator_links(i-length(self.dt_sensor_links))}.last_sampling_time+self.actuator_links{self.dt_actuator_links(i-length(self.dt_sensor_links))}.sampling_time-t;
            end
            
            % DiscreteEventTriggered conditions
            de_conditions = zeros(length(self.de_sensor_links)+length(self.de_actuator_links),1);
            for i = 1:length(self.de_sensor_links)
                de_conditions(i) = self.sensor_links{self.de_sensor_links(i)}.last_sampling_time+self.sensor_links{self.de_sensor_links(i)}.sampling_time-t;
            end
            for i = length(self.de_sensor_links)+1:length(de_conditions)
                de_conditions(i) = self.actuator_links{self.de_actuator_links(i-length(self.de_sensor_links))}.last_sampling_time+self.actuator_links{self.de_actuator_links(i-length(self.de_sensor_links))}.sampling_time-t;
            end
            
            if delta_u > 0
                value = [self.use_delay_events*(self.t_events_delay(end)+delta_u-t); ce_conditions; dt_conditions; de_conditions];
            else
                value = [Inf; ce_conditions; dt_conditions; de_conditions];
            end
            
            value(value < self.event_tol & value > 0) = 0;
            
            isterminal = [1; ones(length(ce_conditions),1); ones(length(dt_conditions),1); ones(length(de_conditions),1)];
            direction = [-1; -ones(length(ce_conditions),1); -ones(length(dt_conditions),1); -ones(length(de_conditions),1)];
        end
        
        function status = on_event(self,ie,te,accumulated_events)
            status = 0;
            
            for i=ie'
                self.te_sim{i} = [self.te_sim{i} te(1)];
            end
            
            n_ce = length(self.ce_sensor_links) + length(self.ce_actuator_links);
            n_dt = length(self.dt_sensor_links) + length(self.dt_actuator_links);
            n_de = length(self.de_sensor_links) + length(self.de_actuator_links);
            
            zeno = 0;
            
            if nargin == 3
                accumulated_events = zeros(1 + n_ce + n_dt + n_de,1);
            end
            
            accumulated_events(ie) = accumulated_events(ie) + 1;
            
            if any(accumulated_events > self.zeno_max_deep)
                % Zeno behaviour detected
                if isempty (self.on_zeno_enter)
                    status = -1;
                    return;
                end
                
                self.on_zeno_enter ();
                zeno = 1;
            end
            
            if any(ie == 1)
                % InputDelay event
                if self.verbose
                    disp('InputDelay Event');
                end
                
                self.t_events_delay = [self.t_events_delay; self.t_sim(end)];
            end
            
            if n_ce > 0
                activated_ce_sensor_links = ie(ie > 1 & ie <= 1 + length(self.ce_sensor_links));
                activated_ce_actuator_links = ie(ie > 1 + length(self.ce_sensor_links) & ie <= 1 + n_ce);
                for i = 1:length(activated_ce_sensor_links)
                    % CE sensor condition
                    if self.verbose
                        disp('ContinuousEventTriggered Sensor Link');
                    end
                    
                    sensor_index = self.ce_sensor_links(activated_ce_sensor_links(i)-1);
                    
                    xp_measured_fun = self.create_signal_fun(self.t_sim,self.xp_measured_sim,PCS.Utils.InterpolationMethod.Linear);
                    y_measured_fun = self.create_signal_fun(self.t_sim,self.y_measured_sim,PCS.Utils.InterpolationMethod.Linear);
                    d_measured_fun = self.create_signal_fun(self.t_sim,self.d_measured_sim,PCS.Utils.InterpolationMethod.Linear);
                    
                    [xp_sent_aux,y_sent_aux,d_sent_aux] = self.sensor_links{sensor_index}.send(self.t_sim(end),xp_measured_fun,y_measured_fun,d_measured_fun);
                    
                    index_aux = ~isnan(xp_sent_aux);
                    self.xp_sent_sim(index_aux,end) = xp_sent_aux(index_aux);
                    
                    index_aux = ~isnan(y_sent_aux);
                    self.y_sent_sim(index_aux,end) = y_sent_aux(index_aux);
                    
                    index_aux = ~isnan(d_sent_aux);
                    self.d_sent_sim(index_aux,end) = d_sent_aux(index_aux);
                end
                for i = 1:length(activated_ce_actuator_links)
                    % CE actuator event
                    if self.verbose
                        disp('ContinuousEventTriggered Actuator Link');
                    end
                    
                    actuator_index = self.ce_actuator_links(activated_ce_actuator_links(i)-1-length(self.ce_sensor_links));
                    
                    u_fun = self.create_signal_fun(self.t_sim,self.u_sim,PCS.Utils.InterpolationMethod.Linear);
                    
                    u_sent_aux = self.actuator_links{actuator_index}.send(self.t_sim(end),u_fun);
                    
                    index_aux = ~isnan(u_sent_aux);
                    self.u_sent_sim(index_aux,end) = u_sent_aux(index_aux);
                end
            end
            
            if n_dt > 0
                activated_dt_sensor_links = ie(ie > 1 + n_ce & ie <= 1 + n_ce + length(self.dt_sensor_links));
                activated_dt_actuator_links = ie(ie > 1 + n_ce + length(self.dt_sensor_links) & ie <= 1 + n_ce + n_dt);
                for i = 1:length(activated_dt_sensor_links)
                    %DT sensor event
                    if self.verbose
                        disp('DiscreteTime Sensor link');
                    end
                    
                    sensor_index = self.dt_sensor_links(activated_dt_sensor_links(i)-1-n_ce);
                    
                    xp_measured_fun = self.create_signal_fun(self.t_sim,self.xp_measured_sim,PCS.Utils.InterpolationMethod.Linear);
                    y_measured_fun = self.create_signal_fun(self.t_sim,self.y_measured_sim,PCS.Utils.InterpolationMethod.Linear);
                    d_measured_fun = self.create_signal_fun(self.t_sim,self.d_measured_sim,PCS.Utils.InterpolationMethod.Linear);
                    
                    [xp_sent_aux,y_sent_aux,d_sent_aux] = self.sensor_links{sensor_index}.send(self.t_sim(end),xp_measured_fun,y_measured_fun,d_measured_fun);
                    
                    index_aux = ~isnan(xp_sent_aux);
                    self.xp_sent_sim(index_aux,end) = xp_sent_aux(index_aux);
                    
                    index_aux = ~isnan(y_sent_aux);
                    self.y_sent_sim(index_aux,end) = y_sent_aux(index_aux);
                    
                    index_aux = ~isnan(d_sent_aux);
                    self.d_sent_sim(index_aux,end) = d_sent_aux(index_aux);
                end
                for i = 1:length(activated_dt_actuator_links)
                    % DT actuator event
                    if self.verbose
                        disp('DiscreteTime Actuator link');
                    end
                    
                    actuator_index = self.dt_actuator_links(activated_dt_actuator_links(i)-1-n_ce-length(self.dt_sensor_links));
                    
                    u_fun = self.create_signal_fun(self.t_sim,self.u_sim,PCS.Utils.InterpolationMethod.Linear);
                        
                    u_sent_aux = self.actuator_links{actuator_index}.send(self.t_sim(end),u_fun);
                        
                    index_aux = ~isnan(u_sent_aux);
                    self.u_sent_sim(index_aux,end) = u_sent_aux(index_aux);
                end
            end
            
            if n_de > 0
                activated_de_sensor_links = ie(ie > 1 + n_ce + n_dt & ie <= 1 + n_ce + n_dt + length(self.de_sensor_links));
                activated_de_actuator_links = ie(ie > 1 + n_ce + n_dt + length(self.de_sensor_links) & ie <= 1 + n_ce + n_dt + n_de);
                for i = 1:length(activated_de_sensor_links)
                    sensor_index = self.de_sensor_links(activated_de_sensor_links(i)-1-n_ce-n_dt);
                    
                    self.sensor_links{sensor_index}.last_sampling_time = self.t_sim(end);
                    
                    xp_measured_fun = self.create_signal_fun(self.t_sim,self.xp_measured_sim,PCS.Utils.InterpolationMethod.ZOH);
                    y_measured_fun = self.create_signal_fun(self.t_sim,self.y_measured_sim,PCS.Utils.InterpolationMethod.ZOH);
                    d_measured_fun = self.create_signal_fun(self.t_sim,self.d_measured_sim,PCS.Utils.InterpolationMethod.ZOH);
                    u_fun = self.create_signal_fun(self.t_sim,self.u_sim,PCS.Utils.InterpolationMethod.Linear);
                    
                    if self.sensor_links{sensor_index}.triggering_condition(self.t_sim(end),self.sensor_links{sensor_index}.last_triggering_time,0,x_measured_fun,y_measured_fun,d_measured_fun,0,u_fun) < 0
                        % DE sensor event
                        if self.verbose
                            disp('DiscreteEventTriggered Sensor link');
                        end
                        
                        [xp_sent_aux,y_sent_aux,d_sent_aux] = self.sensor_links{sensor_index}.send(self.t_sim(end),xp_measured_fun,y_measured_fun,d_measured_fun);

                        index_aux = ~isnan(xp_sent_aux);
                        self.xp_sent_sim(index_aux,end) = xp_sent_aux(index_aux);

                        index_aux = ~isnan(y_sent_aux);
                        self.y_sent_sim(index_aux,end) = y_sent_aux(index_aux);

                        index_aux = ~isnan(d_sent_aux);
                        self.d_sent_sim(index_aux,end) = d_sent_aux(index_aux);
                    else
                        self.te_sim{activated_de_sensor_links(i)}(end) = [];
                    end
                end
                for i = 1:length(activated_de_actuator_links)
                    actuator_index = self.de_actuator_links(activated_de_actuator_links(i)-1-n_ce-n_dt-length(self.de_sensor_links));
                    
                    self.actuator_links{actuator_index}.last_sampling_time = self.t_sim(end);
                    
                    xp_measured_fun = self.create_signal_fun(self.t_sim,self.xp_measured_sim,PCS.Utils.InterpolationMethod.ZOH);
                    y_measured_fun = self.create_signal_fun(self.t_sim,self.y_measured_sim,PCS.Utils.InterpolationMethod.ZOH);
                    d_measured_fun = self.create_signal_fun(self.t_sim,self.d_measured_sim,PCS.Utils.InterpolationMethod.ZOH);
                    u_fun = self.create_signal_fun(self.t_sim,self.u_sim,PCS.Utils.InterpolationMethod.Linear);
                    
                    if self.actuator_links{actuator_index}.triggering_condition(self.t_sim(end),self.actuator_links{actuator_index}.last_triggering_time,0,xp_measured_fun,y_measured_fun,d_measured_fun,0,u_fun) < 0
                        % DE actuator event
                        if self.verbose
                            disp('DiscreteEventTriggered Actuator Link');
                        end
                        
                        u_sent_aux = self.actuator_links{actuator_index}.send(self.t_sim(end),u_fun);
                        
                        index_aux = ~isnan(u_sent_aux);
                        self.u_sent_sim(index_aux,end) = u_sent_aux(index_aux);
                    else
                        self.te_sim{activated_de_actuator_links(i)}(end) = [];
                    end
                end
            end
            
            [~,~,~,u,y] = self.derivatives(self.t_sim(end),[self.xp_sim(:,end);self.xc_sim(:,end)]);
            self.u_sim(:,end) = u;
            self.y_sim(:,end) = y;
            
            ie = find(self.events(self.t_sim(end),[self.xp_sim(:,end);self.xc_sim(:,end)]) <= 0);
            ie = ie(ie ~= 1);
            if isempty(ie)
                if zeno && ~isempty(self.on_zeno_exit)
                    self.on_zeno_exit();
                end
            else
                if zeno
                    status = 1;
                    return;
                end
                status = self.on_event(ie,te,accumulated_events);
            end
        end
        
        function check_integrity(self)
            % Check controller/process constraints
            if isempty(self.process)
                error ('Error: Process is not assigned.');
            end
            
            if isempty(self.controller)
                error ('Error: Controller is not assigned.');
            end
            
            if self.controller.n_outputs ~= self.process.n_inputs
                error ('Error: The number of outputs of the controller does not match the number of inputs of the process.');
            end
            
            % Check initial states constraints
            if size(self.d0,1) ~= self.process.n_disturbances
                error ('Error: Initial measurable disturbances does not match the number of measurable disturbances of the process.');
            end
            
            if size(self.u0,1) ~= self.process.n_inputs
                error ('Error: Initial control signals does not match the number of inputs of the process.');
            end
            
            if size(self.x0,1) ~= self.process.n_states
                error ('Error: Initial states of process does not match the number of states of the process.');
            end
            
            if size(self.y0,1) ~= self.process.n_outputs
                error ('Error: Initial outputs of process does not match the number of outputs of the process.');
            end
        end
        
        function data = end_simulation(self,time_vector)
            if isempty(time_vector)
                data = struct('t',self.t_sim,'x',self.xp_sim,'x_measured',self.xp_measured_sim,'x_sent',self.xp_sent_sim,'y',self.y_sim,'y_measured',self.y_measured_sim,'y_sent',self.y_sent_sim,'xc',self.xc_sim,'u',self.u_sim,'u_sent',self.u_sent_sim,'u_actuator',self.u_actuator_sim,'d',self.d_sim,'d_sent',self.d_sent_sim,'r',self.r_sim,'te',{self.te_sim});
            else
                xp_function = self.create_signal_fun(self.t_sim,self.xp_sim,PCS.Utils.InterpolationMethod.Linear);

                data = struct('t',time_vector,'xp',xp_function(time_vector));
            end
            
            if self.adaptive_initial_step
                self.initial_step = 0;
            end
            
            if self.adaptive_max_step
                self.max_step = 0;
            end
            
            close(self.h_wb,'force');
        end
        
        function init_simulation(self)
            self.preallocate_actuator_links();
            self.preallocate_sensor_links();
            
            self.t_events_delay = self.t0;
            
            self.t_sim = self.t0;
            self.te_sim = cell(1+length(self.ce_sensor_links)+length(self.ce_actuator_links)+length(self.dt_sensor_links)+length(self.dt_actuator_links)+length(self.de_sensor_links)+length(self.de_actuator_links),1);
            for i=1:length(self.te_sim)
                self.te_sim{i} = 0;
            end
            self.xc_sim = self.xc0;
            self.xp_sim = self.x0;
            
            self.d_sim = zeros(self.process.n_disturbances,1);
            for i = 1:self.process.n_disturbances
                self.d_sim(i) = self.d_preloaded_fun{i}(0);
            end
            
            self.r_sim = zeros(self.process.n_outputs,1);
            for i = 1:self.process.n_outputs
                self.r_sim(i) = self.r_preloaded_fun{i}(0);
            end
            
            [~,~,~,u,y,x_measured,y_measured,d_measured,xp_sent,y_sent,d_sent,u_sent,u_actuator] = self.derivatives(self.t0,[self.x0; self.xc0]);
            
            self.u_sim = u;
            self.y_sim = y;
            self.xp_measured_sim = x_measured;
            self.y_measured_sim = y_measured;
            self.d_measured_sim = d_measured;
            self.xp_sent_sim = xp_sent;
            self.y_sent_sim = y_sent;
            self.d_sent_sim = d_sent;
            self.u_sent_sim = u_sent;
            self.u_actuator_sim = u_actuator;
            
            if self.initial_step == 0
                self.adaptive_initial_step = 1;
                
                self.initial_step = 0.1*(self.tend - self.t0);
            end
            
            if self.max_step == 0
                self.adaptive_max_step = 1;
                
                self.max_step = self.tend - self.t0;
            end
            
            if self.show_waitbar
                self.h_wb = waitbar(0,'Initializing...','CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
                setappdata(self.h_wb,'canceling',0);
            end
        end
        
        function status = outputs(self,t,x,flag)
            switch flag
                case 'init'
                    if self.controller.n_states == 0
                        ie = find(self.events(self.t_sim(end),self.xp_sim(:,end)) <= 0);
                    else
                        ie = find(self.events(self.t_sim(end),[self.xp_sim(:,end); self.xc_sim(:,end)]) <= 0);
                    end
                    ie = ie(ie ~= 1);
                    if ~isempty(ie)
                        if self.verbose
                            disp('Event on initial step');
                        end
                        
                        status_zeno = self.on_event(ie,self.t_sim(end));
                        
                        if status_zeno == 1
                            error ('Error: Zeno effect could not be resolved.');
                        elseif status_zeno == -1
                            error('Error: Unhandled zeno exception. Define on_zeno_enter function to handle zeno effect.');
                        end
                        
                        status = 1;
                    end
                    return;
                case 'done'
                    return;
                otherwise
                    for i = 1:length(t)
                        xp = x(1:self.process.n_states,i);
                        xc = x(self.process.n_states+1:end,i);
                        [~,d,r,u,y,xp_measured,y_measured,d_measured,xp_sent,y_sent,d_sent,u_sent,u_actuator] = self.derivatives(t(i),x(:,i));
                        
                        self.d_sim = [self.d_sim d];
                        self.d_measured_sim = [self.d_measured_sim d_measured];
                        self.d_sent_sim = [self.d_sent_sim d_sent];
                        self.t_sim = [self.t_sim t(i)];
                        self.r_sim = [self.r_sim r];
                        self.u_sim = [self.u_sim u];
                        self.u_sent_sim = [self.u_sent_sim u_sent];
                        self.u_actuator_sim = [self.u_actuator_sim u_actuator];
                        self.xp_sim = [self.xp_sim xp];
                        self.xp_measured_sim = [self.xp_measured_sim xp_measured];
                        self.xp_sent_sim = [self.xp_sent_sim xp_sent];
                        self.xc_sim = [self.xc_sim xc];
                        self.y_sim = [self.y_sim y];
                        self.y_measured_sim = [self.y_measured_sim y_measured];
                        self.y_sent_sim = [self.y_sent_sim y_sent];
                    end
                    
                    if self.show_waitbar
                        if getappdata(self.h_wb,'canceling')
                            status = 1;
                            return;
                        else
                            waitbar(t(end)/self.tend,self.h_wb,sprintf('Running... %0.0f%%',100*t(end)/self.tend));
                        end
                    end
            end

            status = 0; % status = 1, halts integration.
        end
        
        function preallocate_actuator_links(self)
            self.ct_actuator_links = [];
            self.dt_actuator_links = [];
            self.ce_actuator_links = [];
            self.de_actuator_links = [];
            
            for i=1:length(self.actuator_links)
                switch self.actuator_links{i}.type
                    case PCS.Network.LinkType.ContinuousTime
                        self.ct_actuator_links = [self.ct_actuator_links i];
                    case PCS.Network.LinkType.DiscreteTime
                        self.dt_actuator_links = [self.dt_actuator_links i];
                    case PCS.Network.LinkType.ContinuousEventTriggered
                        self.ce_actuator_links = [self.ce_actuator_links i];
                    case PCS.Network.LinkType.DiscreteEventTriggered
                        self.de_actuator_links = [self.de_actuator_links i];
                end
            end
        end
        
        function preallocate_sensor_links(self)
            self.ct_sensor_links = [];
            self.dt_sensor_links = [];
            self.ce_sensor_links = [];
            self.de_sensor_links = [];
            
            for i=1:length(self.sensor_links)
                switch self.sensor_links{i}.type
                    case PCS.Network.LinkType.ContinuousTime
                        self.ct_sensor_links = [self.ct_sensor_links i];
                    case PCS.Network.LinkType.DiscreteTime
                        self.dt_sensor_links = [self.dt_sensor_links i];
                    case PCS.Network.LinkType.ContinuousEventTriggered
                        self.ce_sensor_links = [self.ce_sensor_links i];
                    case PCS.Network.LinkType.DiscreteEventTriggered
                        self.de_sensor_links = [self.de_sensor_links i];
                end
            end
        end
        
        function fun = create_signal_fun(self,time_vector,from,interp)
            %% create_signal_fun  Construct a signal function of time based
            %    on current data using an interpolation method.
            %
            %    FUN = create_signal_fun(TIME,FROM,INTERP) constructs a
            %    signal function FUN of TIME based on FROM using an INTERP
            %    method.
            fun = @(varargin) self.value_at_time(time_vector,from,interp,varargin);
        end
        
        function value = value_at_time(~,time_vector,from,interp,index_time)
            if length(index_time) == 1
                % No index specified
                time = index_time{1};
                index = 1:size(from,1);
            else
                % Index was specified
                time = index_time{1};
                index = index_time{2};
            end
            
            if isempty(from)
                value = [];
            else
                value = zeros(length(index),length(time));
                for i = 1:length(time)
                    if ~any(time_vector == time(i))
                        % Inexact value
                        i0 = find(time_vector < time(i),1,'last');
                        iend = find(time_vector > time(i),1,'first');

                        if isempty(i0)
                            % There is no past data
                            value(:,i) = from(index,iend);
                        else
                            switch interp
                                case PCS.Utils.InterpolationMethod.ZOH
                                    % Keep last value
                                    value(:,i) = from(index,i0);
                                case PCS.Utils.InterpolationMethod.Linear
                                    % Linear interpolation
                                    m = (from(index,iend)-from(index,i0))/(time_vector(iend)-time_vector(i0));

                                    value(:,i) = from(index,i0) + m*(time(i)-time_vector(i0));
                                case PCS.Utils.InterpolationMethod.Lagrange
                                    %% TODO
                                    value = [];
                            end
                        end
                    else
                        % Exact value
                        value(:,i) = from(index,find(time_vector == time(i),1,'first'));
                    end
                end
            end
        end
    end
end