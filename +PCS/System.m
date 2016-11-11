classdef (Abstract) System < handle
    %System Abstract representation for a generic system   
    
    properties
        n_states = 0 % Number of internal states of the system
        n_inputs = 0 % Number of inputs of the system
        n_outputs = 0 % Number of outputs of the system
    end
    
    methods (Abstract)
        % Abstract state derivatives function
        vargout = derivatives(self,t,varargin);
        % Abstract output function
        vargout = outputs(self,t,varargin);
    end   
end