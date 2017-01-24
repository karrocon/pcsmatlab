# Process Control Simulations (PCS) for MATLAB

**PCS** is an open-source object-oriented library developed by Departamento de Informática y Automática at UNED, aiming to provide an easy-to-develop framework for process control simulations.

## Table of Contents

- [Introduction](#introduction)
- [Getting Started](#getting-started)
- [Using the Library](#using-the-library)
- [API Reference](#api-reference)

## Introduction
This manual describes the use of Matlab-based simulation library PCS, which facilitates development of process control loops in networked and/or event-based environments.

Several tutorial examples are provided to show how to prepare reusable controllers and processes and perform continuous, periodic or event-triggered simulations.

This manual also includes a detailed API reference of the library with every package and its contents.

For questions and bug reports, please direct them to the [issues page of the library](https://github.com/crcuned/pcsmatlab/issues).

## Getting Started

Download the zip archive from [PCS for MATLAB homepage](https://github.com/crcuned/pcsmatlab). Then

1. Extract all files to some suitable directory.
2. Start MATLAB R2014a or higher and navigate to previous directory.
3. Run *install.m* to add the necessary directories to the MATLAB path.

## Using the Library
Tutorials are given in ascending difficulty order.

### Development of a Quadruple-Tank Process
The following class represents the quadruple-tank process described [here](http://ieeexplore.ieee.org/document/845876/).

~~~matlab
classdef QuadrupleTank < PCS.Process.Process
	properties
		a % Cross-sections of the outlets holes
		A % Cross-sections of the tanks
		g % Acceleration of gravity
		gamma % Positions of the valves
		k % Voltage to volumetric flow rate gains
	end
	
	methods
		function self = QuadrupleTank(a, A, g, gamma, k)
			if ndims(a) ~= 2 || length(a) ~= 4
				error('Cross-sections of the outlets holes must be a vector of length 4.');
			end
			if ndims(A) ~= 2 || length(A) ~= 4
				error('Cross-sections of the tanks must be a vector of length 4.');
			end
			if ~isscalar(g)
				error('Acceleration of gravity must be a scalar.');
			end
			if ndims(gamma) ~= 2 || length(gamma) ~= 2
				error('Positions of the valves must be a vector of length 2.');
			end
			if ndims(k) ~= 2 || length(k) ~= 2
				error('Voltage to volumetric flow rate gains must be a vector of length 2.');
			end
			
			self.a = a;
			self.A = A;
			self.g = g;
			self.gamma = gamma;
			self.k = k;
			
			self.n_inputs = 2;
			self.n_outputs = 2;
			self.n_states = 4;
		end
		
		function dxdt = derivatives(self, t, x, u, ~, ~)
			dxdt = zeros(4, 1);
			
			xt = x(t);
			
			dxdt(1) = -self.a(1)/self.A(1)*sqrt(2*self.g*xt(1)) + self.a(3)/self.A(1)*sqrt(2*self.g*xt(3)) + self.gamma(1)*self.k(1)/self.A(1)*u(t,1);
            dxdt(2) = -self.a(2)/self.A(2)*sqrt(2*self.g*xt(2)) + self.a(4)/self.A(2)*sqrt(2*self.g*xt(4)) + self.gamma(2)*self.k(2)/self.A(2)*u(t,2);
            dxdt(3) = -self.a(3)/self.A(3)*sqrt(2*self.g*xt(3)) + (1-self.gamma(2))*self.k(2)/self.A(3)*u(t,2);
            dxdt(4) = -self.a(4)/self.A(4)*sqrt(2*self.g*xt(4)) + (1-self.gamma(1))*self.k(1)/self.A(4)*u(t,1);
		end
		
		function y = outputs(self, t, x, u, ~, ~)
			y = x(t, [1 2]);
		end
	end
end
~~~

### Development of a PI Controller
The following class represents a general PI controller.

~~~matlab
classdef PI < PCS.Control.Controller
	properties
		K % Proportional gains
		Ti % Integral times
	end
	
	methods
		function self = PI(K, Ti)
			if ndims(K) > 2
				error('Proportional gains must be a vector.');
			end
			if ndims(Ti) > 2
				error('Integral times must be a vector.');
			end
			if length(K) ~= length(Ti)
				error('The lengths of the proportional gains and integral times must match.');
			end
			
			self.K = K;
			self.Ti = Ti;
			
			self.n_inputs = length(K);
			self.n_outputs = length(K);
			self.n_states = length(K);
		end
		
		function dxcdt = derivatives(self, t, ~, ~, y, ~, r)
			dxcdt = self.Ti.^(-1).*(r(t) - y(t));
		end
		
		function u = outputs(self, t, xc, ~, y, ~, r)
			u = self.K.*xc(t) + self.K.*(r(t) - y(t));
		end
	end
end
~~~

### Executing a Simulation
The following script is used to perform the [PI control](#development-of-a-pi-controller) of the [quadruple-tank process](#development-of-a-quadruple-tank-process).

~~~matlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PI control of a quadruple-tank process %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Quadruple-tank process
a = [0.071 0.057 0.071 0.057];
A = [28 32 28 32];
g = 981;
gamma = [0.7 0.6];
k = [3.33 3.35];

process = QuadrupleTank(a, A, g, gamma, k);

% PI controller
K = [0.3816; 0.5058];
Ti = [62.9557; 91.3960];

controller = PI(K, Ti);

% Create simulation
simulation = PCS.Simulation(controller, process);

% Define initial states and time interval
simulation.xc0 = [31.4347; 33.4446];
simulation.x0 = [12.4; 12.7; 1.5919; 1.4551];
simulation.t0 = 0;
simulation.tend = 500;

% Define set-point conditions
simulation.set_preloaded_reference(1, 15);
simulation.set_preloaded_reference(2, 12.7);

% Execute simulation
data = simulation.run();

% Plot results
plot(data.t, data.x);
~~~

## API Reference
The API reference is organized by packages.

| Package                       | Description                                                                                                  |
|:----------------------------- |:------------------------------------------------------------------------------------------------------------ |
| [PCS](#pcs)                   | Provides the core classes necessary to perform a process control simulation and the base system definition.  |
| [PCS.Control](#pcscontrol)   | Provides the base controller definition and some of the well-known control laws.                             |
| [PCS.Hardware](#pcshardware) | Provides classes for emulating physical interaction with real processes.                                     |
| [PCS.Network](#pcsnetwork)   | Provides classes for establishing a networked communication between control loop components.                 |
| [PCS.Process](#pcsprocess)   | Provides the base process definition and some of the well-known industrial processes.                        |
| [PCS.Utils](#pcsutils)       | Provides static classes with useful reusable functions to simplify the simulation of the closed-loop system. |

### PCS
Provides the core classes necessary to perform a process control simulation and the base system definition.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Simulation](#class-simulation-pcs) | The *Simulation* class contains the core functionality of the library that allows to perform process control simulations. |
| [System](#class-system-pcs) | An abstract class which provides a common definition for processes and controllers.                                                 |

##### Class Simulation ([PCS](#pcs))
- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

The *Simulation* class contains the core functionality of the library that allows to perform process control simulations.

###### Properties Summary
| Modifier | Property  | Description |
|:-------- |:--------- |:----------- |
| None | actuators | Vector containing an actuator for each input of the process. |
| None | actuator\_links | Vector containing the links of the network controller-process (C-P). |
| None | d\_sensors | Vector containing a sensor for each disturbance affecting the process. |
| None | event\_tol | Zero-crossing tolerance for event location. |
| None | on\_zeno\_enter | Function handle to be called before a Zeno phenomenom occurs. |
| None | on\_zeno\_exit | Function handle to be called after Zeno phenomenom is solved. |
| None | sensor\_links | Vector containing the links of the network process-controller (P-C). |
| None | solver | A function handle to the ODE solver. |
| None | t0 | The initial time of the simulation. |
| None | tend | The final time of the simulation. |
| None | verbose | True if debug messages should be displayed at runtime. |
| None | x0 | Initial states of the process. |
| None | xc0 | Initial states of the controller. |
| None | x\_sensors | Vector containing a sensor for each state of the process. |
| None | y\_sensors | Vector containing a sensor for each output of the process. |
| None | zeno\_max\_depth | Maximum times that the same event can be triggered in the same time instant before a Zeno effect is detected. |

###### Constructors Summary
| Constructor | Description |
|:----------- |:----------- |
| Simulation(process, controller) | Initializes a closed-loop simulation with given process and controller. |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| None     | data = run() | Executes the simulation in the time interval [t0, tend]. |
| None     | data = run(t) | Executes the simulation in the time interval [t0, tend] and returns the result at prefixed time instants given by t. |

##### Class System ([PCS](#pcs))
- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

An abstract class which provides a common definition for processes and controllers.

###### Properties Summary
| Modifier | Property  | Description |
|:-------- |:--------- |:----------- |
| None     | n_inputs  | The number of inputs of the system.  |
| None     | n_outputs | The number of outputs of the system. |
| None     | n_states  | The number of states of the system.  |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Abstract | [varargout](https://es.mathworks.com/help/matlab/ref/varargout.html) = derivatives(t, [varargin](https://es.mathworks.com/help/matlab/ref/varargin.html)) | Computes the derivatives of the states of the system. |
| Abstract | [varargout](https://es.mathworks.com/help/matlab/ref/varargout.html) = outputs(t, [varargin](https://es.mathworks.com/help/matlab/ref/varargin.html)) | Computes the outputs of the system. |

### PCS.Control
Provides the base controller definition and some of the well-known control laws.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Controller](#class-controller-pcscontrol) | An abstract class which provides a general definition for controllers. |

##### Class Controller ([PCS.Control](#pcscontrol))
- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

	- [PCS.System](#class-system-pcs)

An abstract class which provides a general definition for controllers.

###### Method Summary

| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Abstract | dxcdt = derivatives(t, xc, x, y, d, r) | Computes the derivatives of the states of the controller. |
| Abstract | u = outputs(t, xc, x, y, d, r) | Computes the control signals. |

### PCS.Hardware
Provides classes for emulating physical interaction with real processes.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Actuator](#class-actuator-pcshardware) | This class represents a physical actuator that is continuously connected to some process input. |
| [Device](#class-device-pcshardware) | An abstract class which provides a general definition for an electronic device. |
| [Sensor](#class-sensor-pcshardware) | This class represents a physical sensor that is continuously connected to some process state or output, or to a disturbance. |

##### Class Actuator ([PCS.Hardware](#pcshardware))
- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

	- [PCS.Hardware.Device](#class-device-pcshardware)

This class represents a physical actuator that is continuously connected to some process input.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | dead_band | The dead band of the actuator at which it does not provide any control action to the process. |
| None | range | The range of the actuator that is used for saturation. |
| None | resolution | The resolution of the actuator for rounding purposes. |
| None | slew_rate | The limitation in speed change of the actuator. |

###### Constructors Summary
| Constructor | Description |
|:----------- |:----------- |
| Actuator()  | Creates an ideal actuator with no physical restrictions. |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| None     | output = write(input) | Uses this actuator to write a (possibly constrained) control signal. |

##### Class Device ([PCS.Hardware](#pcshardware))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

An abstract class which provides a general definition for an electronic device.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | device_id | The identifier of the physical device. |

##### Class Sensor ([PCS.Hardware](#pcshardware))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [PCS.Hardware.Device](#class-device-pcshardware)

This class represents a physical sensor that is continuously connected to some process state or output, or to a disturbance.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None     | accuracy | The sensor accuracy within the range (0: noisy, 1: exact]. |
| None | range | The imposed range of the measured signal. |
| None | resolution | The resolution of the sensor for rounding purposes. |

###### Constructors Summary
| Constructor | Description |
|:----------- |:----------- |
| Sensor()    | Creates an ideal sensor with no physical restrictions. |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| None     | output = read(input) | Uses this sensor to read a (possibly constrained) state, output or disturbance. |

### PCS.Network
Provides classes for establishing a networked communication between control loop components.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [ActuatorLink](#class-actuatorlink-pcsnetwork) | The *ActuatorLink* class provides a general implementation for continuous-time, discrete-time and event-time transmissions between the controller and the process. |
| [Link](#class-link-pcsnetwork) | An abstract class which provides a general definition for network links. |
| [SensorLink](#class-sensorlink-pcsnetwork) | The *SensorLink* class provides a general implementation for continuous-time, discrete-time and event-time transmissions between the process and the controller. |

#### Enum Summary
| Enum  | Description |
|:----- |:----------- |
| [LinkType](#enum-link-pcsnetwork) | The available types of links for networked communication.|

##### Class ActuatorLink ([PCS.Network](#pcsnetwork))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [Link](#class-link-pcsnetwork)

The *ActuatorLink* class provides a general implementation for continuous-time, discrete-time and event-time transmissions between the controller and the process.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | control\_signals\_to\_transmit | A vector containing the index of the transmitted control signals. |

###### Constructors Summary
| Constructor | Description |
|:----------- |:----------- |
| ActuatorLink() | Creates a continuous actuator link that transmits every control signal. |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| None | u_sent = send(t, u) | Transmits some control signals through this link. |

##### Class Link ([PCS.Network](#pcsnetwork))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

An abstract class which provides a general definition for network links.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | reliability | The reliability of the link that may cause packet losses in the range [0: all packets are lost, 1: no packet losses]. |
| None | sampling_time | The periodic sampling time of the link in the range [0: continuous transmission, &infin;). |
| None | transmission_delay | The transmission delay of the link. |
| None | triggering_condition | The event condition that must be fulfilled for the link to be triggered. |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| None | [varargout](https://es.mathworks.com/help/matlab/ref/varargout.html) = send([varargin](https://es.mathworks.com/help/matlab/ref/varargin.html)) | Transmits some signals through this link. |

##### Class SensorLink ([PCS.Network](#pcsnetwork))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [Link](#class-link-pcsnetwork)

The *SensorLink* class provides a general implementation for continuous-time, discrete-time and event-time transmissions between the process and the controller.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | disturbances\_to\_transmit | A vector containing the index of the transmitted disturbances. |
| None | outputs\_to\_transmit | A vector containing the index of the transmitted outputs. |
| None | states\_to\_transmit | A vector containing the index of the transmitted states. |

###### Constructors Summary
| Constructor | Description |
|:----------- |:----------- |
| SensorLink() | Creates a continuous sensor link that transmits every disturbance, output and state. |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| None | [x\_sent, y\_sent, d\_sent] = send(t, x, y, d) | Transmits some disturbances, outputs and states through this link. |

##### Enum LinkType ([PCS.Network](#pcsnetwork))
The available types of links for networked communication.

###### Constants Summary
| Constant                 | Description |
|:-----------------------  |:----------- |
| ContinuousEventTriggered | Represents a continuous event-triggered link. | 
| ContinuousTimeTriggered  | Represents a continuous link. |
| PeriodicEventTriggered   | Represents a periodic event-triggered link. |
| PeriodicTimeTriggered    | Represents a periodic time-triggered link. |

### PCS.Process
Provides the base process definition and some of the well-known industrial processes.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Process](#class-process-pcsprocess) | An abstract class which provides a general definition for processes. |

##### Class Process ([PCS.Process](#pcsprocess))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [System](#class-system-pcs)

An abstract class which provides a general definition for processes.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | n_disturbances | The number of disturbances of the process. |

###### Method Summary

| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Abstract | dxdt = derivatives(t, x, u, d) | Computes the derivatives of the states of the process. |
| Abstract | y = outputs(t, x, u, d) | Computes the outputs of the process. |

### PCS.Utils
Provides static classes with useful reusable functions to simplify the simulation of the closed-loop system.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Utils](#class-utils-pcsutils) | Class for static utilities. |

#### Enum Summary
| Enum                | Description |
|:------------------- |:----------- |
| [InterpolationMethod](#enum-interpolationmethod-pcsutils) | The avaliable modes of interpolation. |

##### Class Utils ([PCS.Utils](#pcsutils))

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Static   | y = langrange_interpolation(X, Y, x) | Computes Lagrange interpolation. |
| Static | classes = subclasses(class, folder, depth) | Gets MATLAB subclasses for a given class. |

##### Enum InterpolationMethod ([PCS.Utils](#pcsutils))
The available modes of interpolation.

###### Constants Summary
| Constant | Description |
|:-------- |:----------- |
| Lagrange | The Lagrange interpolation method. |
| Linear | Linear (first-order hold) interpolation. |
| ZOH | Zero-order hold interpolation. |