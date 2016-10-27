classdef ActuatorLink < PCS.Network.Link
    
    properties
        control_signals_to_transmit
    end
    
    %%%%%%%%%%%%%%%
    % CONSTRUCTOR %
    %%%%%%%%%%%%%%%
    methods
        function self = ActuatorLink()
            self.send_delegate = @self.send_continuous_time;
        end
    end
    
    %%%%%%%%%%%
    % GET/SET %
    %%%%%%%%%%%
    methods
        function set.control_signals_to_transmit(self,control_signal_indexes)
            if any(round(control_signal_indexes) ~= control_signal_indexes) || any(control_signal_indexes <= 0)
                error ('Error: Indexes must be positive integers.');
            end
            self.control_signals_to_transmit = control_signal_indexes;
        end
    end
    
    %%%%%%%%%%
    % PUBLIC %
    %%%%%%%%%%
    methods
        function u_sent = send(self,t,u)
            u_sent = self.send_delegate(t,u,self.control_signals_to_transmit);
        end
    end
end

