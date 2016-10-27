classdef (Abstract) Link < handle
        
    properties
        transmission_delay = @(t) 0
        reliability = 1
        last_sampling_time = 0
        sampling_time = 0
        last_triggering_time = 0
        triggering_condition = []
    end
    
    properties (SetAccess = private, Hidden = true)
        type
    end
    
    properties (Access = protected)
        send_delegate
    end
    
    %%%%%%%%%%%
    % GET/SET %
    %%%%%%%%%%%
    methods
        function sampling_time = get.sampling_time(self)
            sampling_time = self.sampling_time;
        end
        
        function set.sampling_time(self,sampling_time)
            if ~isscalar(sampling_time)
                error ('Error: The sampling time must be a positive real number.');
            elseif sampling_time < 0 || ~isnumeric(sampling_time)
                error ('Error: The sampling time must be a positive real number.');
            end
            
            if sampling_time == 0
                if isempty(self.triggering_condition)
                    self.send_delegate = @self.send_continuous_time;
                else
                    self.send_delegate = @self.send_continuous_event_triggered;
                end
            else
                if isempty(self.triggering_condition)
                    self.send_delegate = @self.send_discrete_time;
                else
                    self.send_delegate = @self.send_discrete_event_triggered;
                end
            end
            
            self.sampling_time = sampling_time;
        end
        
        function type = get.type(self)
            if self.sampling_time == 0
                if isempty(self.triggering_condition)
                    type = PCS.Network.LinkType.ContinuousTime;
                else
                    type = PCS.Network.LinkType.ContinuousEventTriggered;
                end
            else
                if isempty(self.triggering_condition)
                    type = PCS.Network.LinkType.DiscreteTime;
                else
                    type = PCS.Network.LinkType.DiscreteEventTriggered;
                end
            end
        end
        
        function set.transmission_delay(self,delay)
            switch class(delay)
                case 'double'
                    if delay < 0
                        error ('Error: Transmission delay must be a real number greater or equal than 0.');
                    end
                    self.transmission_delay = @(~) delay;
                case 'function_handle'
                    self.transmission_delay = delay;
                otherwise
                    error ('Error: Transmission delay must be a real number greater or equal than 0 or a function handle.');
            end
        end
        
        function set.triggering_condition(self,tc)
            if isempty(tc)
                self.triggering_condition = [];
                
                if self.sampling_time == 0
                    self.send_delegate = @self.send_continuous_time;
                else
                    self.send_delegate = @self.send_discrete_time;
                end
            elseif isa(tc,'function_handle')
                if nargin(tc) ~= -1 && nargin(tc) ~= 8
                    error ('Error: The input arguments of the triggering condition should be @(varargin) or @(t,tk,xc,x,y,d,r,u).');
                end
                
                if self.sampling_time == 0
                    self.send_delegate = @self.send_continuous_event_triggered;
                else
                    self.send_delegate = @self.send_discrete_event_triggered;
                end
                
                self.triggering_condition = tc;
            else
                error ('Error: Triggering condition must be a function handle or empty.');
            end
        end
    end
    
    %%%%%%%%%%
    % PUBLIC %
    %%%%%%%%%%
    methods (Abstract)
        %function [xp_sent,y_sent,d_sent] = send(self,t,xp,y,d)
        %    [xp_sent,y_sent,d_sent] = self.send_delegate(t,xp,y,d);
        %end
        varargout = send(self,varargin);
    end
    
    %%%%%%%%%%%%%
    % PROTECTED %
    %%%%%%%%%%%%%
    methods (Access = protected)
        function varargout = send_continuous_event_triggered(self,t,varargin)
            [varargout{1:nargout}] = self.send_continuous_time(t,varargin{:});
            
            self.last_triggering_time = t;
        end
        
        function varargout = send_continuous_time(self,t,varargin)
            delay = self.transmission_delay(t);
            
            varargout = cell(length(varargin)/2,1);
            for i=1:2:length(varargin)
                var_aux = varargin{i}(t-delay);
                
                varargout{(i+1)/2} = NaN*ones(length(var_aux),1);
                
                varargout{(i+1)/2}(varargin{i+1}) = var_aux(varargin{i+1});
            end
        end
        
        function varargout = send_discrete_event_triggered(self,t,varargin)
            if self.reliability - rand() >= 0
                % Packet successfully arrived
                [varargout{1:nargout}] = self.send_continuous_event_triggered(t,varargin{:});
            else
                % Packet lost
                varargout = cell(length(varargin)/2,1);
                for i=1:length(varargout)
                    varargout{i} = NaN;
                end
            end
            
            self.last_triggering_time = t;
            self.last_sampling_time = t;
        end
        
        function varargout = send_discrete_time(self,t,varargin)
            if self.reliability - rand() >= 0
                % Packet successfully arrived
                [varargout{1:nargout}] = self.send_continuous_time(t,varargin{:});
            else
                % Packet lost
                varargout = cell(length(varargin)/2,1);
                for i=1:length(varargout)
                    varargout{i} = NaN;
                end
            end
            
            self.last_sampling_time = t;
        end
    end
end