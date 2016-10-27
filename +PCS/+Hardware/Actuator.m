classdef Actuator < PCS.Hardware.Device
    %Actuator Class that represents a physical actuator.
    %
    %    See also ACTUATOR.ACTUATOR.
    
    properties
        dead_band = @() [0, 0];
        range = [-Inf, Inf]
        resolution = 1e-6
        slew_rate = [-Inf, Inf]
    end
    
    properties (Access = private)
        write_delegate
    end
    
    %%%%%%%%%%%%%%%
    % CONSTRUCTOR %
    %%%%%%%%%%%%%%%
    methods
        function self = Actuator()
            % Actuator  Construct a physical actuator.
            %    A = Actuator() constructs a physical actuator A with  
            %    default parameters.
            %    
            %    See also SENSOR.
            self.write_delegate = @self.write_exact;
        end
    end
    
    %%%%%%%%%%%
    % GET/SET %
    %%%%%%%%%%%
    methods
        function range = get.range(self)
            range = self.range;
        end
        
        function set.range(self,range)
            self.range = range;
            
            if self.is_exact()
                self.write_delegate = @self.write_exact;
            else
                self.write_delegate = @self.write_inexact;
            end
        end
    end
    
    %%%%%%%%%%
    % PUBLIC %
    %%%%%%%%%%
    methods
        function output = write(self,input)
            % Actuator.write  Write the transmitted control signals.
            %    OUT = Actuator.write(IN) writes the given control signal 
            %    IN into the actuator to obtain the final signal OUT that
            %    is applied to the process.
            output = self.write_delegate(input);
        end
    end
    
    %%%%%%%%%%%
    % PRIVATE %
    %%%%%%%%%%%
    methods (Access = private)
        function exact = is_exact(self)
            exact = self.range(1) == -Inf && self.range(2) == Inf;
        end
        
        function output = write_exact(~,input)
            output = input;
        end
        
        function output = write_inexact(self,input)
            % Apply saturation
            output = min(max(input,self.range(1)),self.range(2));
        end
    end
end