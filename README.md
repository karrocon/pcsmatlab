# Process Control Simulations (PCS) for MATLAB

**PCS** is an open-source object-oriented library developed by Departamento de Informática y Automática at UNED, aiming to provide an easy-to-develop framework for process control simulations.

## Table of Contents

- [Introduction](#introduction)
- [Getting Started](#getting-started)
- [Using the Library](#using-the-library)
- [API Reference](#api-reference)

## Introduction
This manual describes the use of Matlab-based simulation library PCS, which facilitates development of process control loops in networked and/or event-based environments.

Several tutorial examples are provided.

## Getting Started

Download the zip archive from [PCS for MATLAB homepage](https://github.com/crcuned/pcsmatlab). Then

1. Extract all files to some suitable directory.
2. Start MATLAB R2014a or higher and navigate to previous directory.
3. Run *install.m* to add the necessary directories to the MATLAB path.

## Using the Library
Under development.

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

##### Class Simulation (PCS)
The *Simulation* class contains the core functionality of the library that allows to perform process control simulations.

##### Class System (PCS)
[handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

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

##### Class Utils ([PCS.Utils](#pcsutils)]

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
| ZOH | Zero-order hold interpolation. |