classdef (Abstract) Process < PCS.System
    %Plant Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        input_delay = @(t) 0;
        disturbance_delay = @(t) 0;
        n_disturbances = 0
    end
    
    methods (Abstract)
        dxdt = derivatives(self,t,x,u,d);
        
        y = outputs(self,t,x,u,d);
    end   
end