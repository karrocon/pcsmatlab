classdef SensorLink < PCS.Network.Link
    
    properties
        states_to_transmit = []
        outputs_to_transmit = []
        disturbances_to_transmit = []
    end
    
    %%%%%%%%%%%%%%%
    % CONSTRUCTOR %
    %%%%%%%%%%%%%%%
    methods
        function self = SensorLink()
            self.send_delegate = @self.send_continuous_time;
        end
    end
    
    %%%%%%%%%%%
    % GET/SET %
    %%%%%%%%%%%
    methods
        function set.disturbances_to_transmit(self,disturbance_indexes)
            if any(round(disturbance_indexes) ~= disturbance_indexes) || any(disturbance_indexes <= 0)
                error ('Error: Indexes must be positive integers.');
            end
            self.disturbances_to_transmit = disturbance_indexes;
        end
        
        function set.outputs_to_transmit(self,output_indexes)
            if any(round(output_indexes) ~= output_indexes) || any(output_indexes <= 0)
                error ('Error: Indexes must be positive integers.');
            end
            self.outputs_to_transmit = output_indexes;
        end
        
        function set.states_to_transmit(self,state_indexes)
            if any(round(state_indexes) ~= state_indexes) || any(state_indexes <= 0)
                error ('Error: Indexes must be positive integers.');
            end
            self.states_to_transmit = state_indexes;
        end
    end
    
    %%%%%%%%%%
    % PUBLIC %
    %%%%%%%%%%
    methods
        function [xp_sent,y_sent,d_sent] = send(self,t,xp,y,d)
            [xp_sent,y_sent,d_sent] = self.send_delegate(t,xp,self.states_to_transmit,y,self.outputs_to_transmit,d,self.disturbances_to_transmit);
            
            %xp_sent = NaN*ones(length(xp),1);
            %y_sent = NaN*ones(length(y),1);
            %d_sent = NaN*ones(length(d),1);
            
            %[xp_sent(self.states_to_transmit),y_sent(self.outputs_to_transmit),d_sent(self.measurable_disturbances_to_transmit)] = self.send_delegate(t,@(t)xp(t,self.states_to_transmit),@(t)y(t,self.outputs_to_transmit),@(t)d(t,self.measurable_disturbances_to_transmit));
        end
    end
end

