classdef (Abstract) Controller < PCS.System
    %Controller  of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods (Abstract)
        dxcdt = derivatives(self,t,xc,x,y,d,r);
        u = outputs(self,t,xc,x,y,d,r);
    end 
end