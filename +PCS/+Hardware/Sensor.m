classdef Sensor < PCS.Hardware.Device
    %Sensor Class that represents a physical sensor.
    
    properties
        accuracy = 1 % Sensor accuracy within the range (0,1] (0: noisy, 1: exact)
        range = [-Inf,Inf] % Measured signal range in format [min_value,max_value]
        resolution = 0 % Sensor resolution
    end
    
    properties (Access = private)
        read_delegate
    end
    
    %%%%%%%%%%%%%%%
    % CONSTRUCTOR %
    %%%%%%%%%%%%%%%
    methods
        function self = Sensor()
            % Sensor  Construct a physical sensor.
            %    S = Sensor() constructs a physical sensor S with default 
            %    parameters.
            %    
            %    See also ACTUATOR.
            self.read_delegate = @self.read_exact;
        end
    end
    
    %%%%%%%%%%%
    % GET/SET %
    %%%%%%%%%%%
    methods
        function accuracy = get.accuracy(self)
            accuracy = self.accuracy;
        end
        
        function set.accuracy(self,accuracy)
            if accuracy <= 0 || accuracy > 1
                error ('Error: Sensor accuracy must be within the range (0,1].');
            end
                        
            self.accuracy = accuracy;
            
            if self.is_exact()
                self.send_delegate = @send_exact;
            else
                self.send_delegate = @send_inexact;
            end
        end
        
        function range = get.range(self)
            range = self.range;
        end
        
        function set.range(self,range)
            if length(range) ~= 2
                error ('Error: Sensor range must be a vector with two scalars.');
            elseif range(1) >= range(2)
                error ('Error: Sensor minimum range value must be lower than maximum range value.');
            end
            
            self.range = range;
            
            if self.is_exact()
                self.send_delegate = @send_exact;
            else
                self.send_delegate = @send_inexact;
            end
        end
        
        function resolution = get.resolution(self)
            resolution = self.resolution;
        end
        
        function set.resolution(self,resolution)
            if resolution < 0
                error ('Error: Sensor resolution must greater or equal than zero.');
            end
                        
            self.resolution = resolution;
            
            if self.is_exact()
                self.send_delegate = @send_exact;
            else
                self.send_delegate = @send_inexact;
            end
        end
    end
    
    methods
        function output = read(self,input)
            output = self.read_delegate(input);
        end
    end
    
    methods (Access = private)
        function exact = is_exact(self)
            exact = self.range(1) == -Inf && self.range(2) == Inf && self.accuracy == 1 && self.resolution == 0;
        end
        
        function output = read_exact(~,input)
            output = input;
        end
        
        function output = read_inexact(self,input)
            % Apply accuracy error
            output = input + (1-self.accuracy)*(rand()-0.5);
            
            % Apply resolution
            output = round(output/self.resolution) * self.resolution;
            
            % Apply saturation
            output = min(max(output,self.range(1)),self.range(2));
        end
    end
end