# Process Control Simulations (PCS) for MATLAB®

**PCS** is an open-source object-oriented library developed by Departamento de Informática y Automática at UNED, aiming to provide an easy-to-develop framework for process control simulations.

## Table of Contents

- [Introduction](#introduction)
- [Getting Started](#getting-started)
- [Using the Library](#using-the-library)
- [API Reference](#api-reference)

## <span id="introduction"/>Introduction
This manual describes the use of Matlab-based simulation library PCS, which facilitates development of process control loops in networked and/or event-based environments.

Several tutorial examples are provided.

## <span id="getting-started"/>Getting Started

Download the zip archive from [PCS for MATLAB homepage](https://github.com/crcuned/pcsmatlab). Then

1. Extract all files to some suitable directory.
2. Start MATLAB R2014a or higher and navigate to previous directory.
3. Run *install.m* to add the necessary directories to the MATLAB path.

## Using the Library
Under development.

## <span id="api-reference"/>API Reference
The API reference is organized by packages.

| Package                       | Description                                                                                                  |
|:----------------------------- |:------------------------------------------------------------------------------------------------------------ |
| [PCS](#pcs)                   | Provides the core classes necessary to perform a process control simulation and the base system definition.  |
| [PCS.Control](#pcscontrol)   | Provides the base controller definition and some of the well-known control laws.                             |
| [PCS.Hardware](#pcshardware) | Provides classes for emulating physical interaction with real processes.                                     |
| [PCS.Network](#pcsnetwork)   | Provides classes for establishing a networked communication between control loop components.                 |
| [PCS.Process](#pcsprocess)   | Provides the base process definition and some of the well-known industrial processes.                        |
| [PCS.Utils](#pcsutils)       | Provides static classes with useful reusable functions to simplify the simulation of the closed-loop system. |

### <span id="pcs"/>PCS
Provides the core classes necessary to perform a process control simulation and the base system definition.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Simulation](#simulation) | The *Simulation* class contains the core functionality of the library that allows to perform process control simulations. |
| [System](#system) | An abstract class which provides a common definition for processes and controllers.                                                 |

##### <span id="simulation"/>Class Simulation (PCS)
The *Simulation* class contains the core functionality of the library that allows to perform process control simulations.

##### <span id="system"/>Class System (PCS)
[handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

An abstract class which provides a common definition for processes and controllers.

###### Properties Summary
| Property  | Description |
|:--------- |:----------- |
| n_inputs  | The number of inputs of the system.  |
| n_outputs | The number of outputs of the system. |
| n_states  | The number of states of the system.  |

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Abstract | [varargout](https://es.mathworks.com/help/matlab/ref/varargout.html) = derivatives(t, [varargin](https://es.mathworks.com/help/matlab/ref/varargin.html)) | Computes the derivatives of the states of the system. |
| Abstract | [varargout](https://es.mathworks.com/help/matlab/ref/varargout.html) = outputs(t, [varargin](https://es.mathworks.com/help/matlab/ref/varargin.html)) | Computes the outputs of the system. |

### <span id="pcs-control"/>PCS.Control
Provides the base controller definition and some of the well-known control laws.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Controller](#controller) | An abstract class which provides a general definition for controllers. |

##### Class Controller ([PCS.Control](#pcscontrol))
- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

	- [PCS.System](#system)

An abstract class which provides a general definition for controllers.

###### Method Summary

| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Abstract | dxcdt = derivatives(t, xc, x, y, d, r) | Computes the derivatives of the states of the controller. |
| Abstract | u = outputs(t, xc, x, y, d, r) | Computes the control signals. |

### <span id="pcs-hardware"/>PCS.Hardware
Provides classes for emulating physical interaction with real processes.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Actuator](#actuator) | This class represents a physical actuator that is continuously connected to some process input. |
| [Device](#device) | An abstract class which provides a general definition for an electronic device. |
| [Sensor](#sensor) | This class represents a physical sensor that is continuously connected to some process state or output, or to a disturbance. |

##### <span id="actuator"/>Class Actuator ([PCS.Hardware](#pcshardware))
- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

	- [PCS.Hardware.Device](#device)

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
| Method | Description |
|:------ |:----------- |
| output = write(input) | Uses this actuator to write a (possibly constrained) control signal. |

##### <span id="device"/>Class Device ([PCS.Hardware](#pcshardware))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)

An abstract class which provides a general definition for an electronic device.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | device_id | The identifier of the physical device. |

##### <span id="sensor"/>Class Sensor ([PCS.Hardware](#pcshardware))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [PCS.Hardware.Device](#device)

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
| Method | Description |
|:------ |:----------- |
| output = read(input) | Uses this sensor to read a (possibly constrained) state, output or disturbance. |

### <span id="pcs-network"/>PCS.Network
Provides classes for establishing a networked communication between control loop components.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [ActuatorLink](#actuator-link) | The *ActuatorLink* class provides a general implementation for continuous-time, discrete-time and event-time transmissions between the controller and the process. |
| [Link](#link) | An abstract class which provides a general definition for network links. |
| [SensorLink](#sensor-link) | The *SensorLink* class provides a general implementation for continuous-time, discrete-time and event-time transmissions between the process and the controller. |

##### <span id="actuator-link"/>Class ActuatorLink ([PCS.Network](#pcsnetwork))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [Link](#link)

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

##### <span id="link"/>Class Link ([PCS.Network](#pcsnetwork))

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

##### <span id="sensor-link"/>Class SensorLink ([PCS.Network](#pcsnetwork))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [Link](#link)

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

### <span id="pcs-process"/>PCS.Process
Provides the base process definition and some of the well-known industrial processes.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| [Process](#process) | An abstract class which provides a general definition for processes. |

##### <span id="process"/>Class Process ([PCS.Process](#pcsprocess))

- [handle](https://es.mathworks.com/help/matlab/ref/handle-class.html)
	- [System](#system)

An abstract class which provides a general definition for processes.

###### Properties Summary
| Modifier | Property | Description |
|:-------- |:-------- |:----------- |
| None | n_disturbances | The number of disturbances of the process |

###### Method Summary

| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Abstract | dxdt = derivatives(t, x, u, d) | Computes the derivatives of the states of the process. |
| Abstract | y = outputs(t, x, u, d) | Computes the outputs of the process. |

### <span id="pcs-utils"/>PCS.Utils
Provides static classes with useful reusable functions to simplify the simulation of the closed-loop system.

#### Class Summary
| Class      | Description |
|:---------- |:----------- |
| Utils | Class for static utilities. |

##### <span id="utils"/>Class Utils ([PCS.Utils](#pcsutils)]

###### Methods Summary
| Modifier | Method | Description |
|:-------- |:------ |:----------- |
| Static   | y = langrange_interpolation(X, Y, x) | Computes Lagrange interpolation. |
| Static | classes = subclasses(class, folder, depth) | Gets MATLAB subclasses for a given class. |