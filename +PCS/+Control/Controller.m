classdef (Abstract) Controller < PCS.System
    %Controller  of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods (Abstract)
        dxdt = derivatives(self,t,xc,x,y,d,r);
        u = output(self,t,xc,x,y,d,r);
    end 
end