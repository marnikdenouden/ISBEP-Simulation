ISBEP Robot Simulation
=============================

 - [Overview](#overview)
 - [Features](#features)
   - [Settings](#settings)
   - [Communicator](#communicator)
   - [Timeline](#timeline)
   - [Situation](#situation)
   - [Util](#util)
 - [Simulation installation](#simulation-installation)
 - [Simulation usage](#simulation-usage)
 - [Dependencies](#dependencies)

-----------------------------

# Overview
Robot simulation for the Innovation Space Bachelor End Project ([ISBEP](https://studiegids.tue.nl/opleidingen/innovation-space/bachelor/isbep-innovation-space-bachelor-end-project))

# Features
[This]() repository contains a multi-agent exploration simulation that was created by an electrical engineering student during the ISBEP project. The features described for this repository consider the technical features that can be used for integrating the simulation with the [ISBEP-Server](https://github.com/marnikdenouden/ISBEP-Server).

## Settings
The [settings.py](settings.py) file contains most of the parameters that can be altered for the simulation and the accompanying features of [this]() repository. For example in this file you can specify where to export simulation data to by changing the export parameters.

## Communicator
In order to communicate data from the simulation to the server a TCP client is created. In the [settings](#settings) the connection can be enabled/disabled and the port and address can be set. The [connection](connection) folder contains the protocol for creating the TCP client, which ensure messages from [communicator.py](communicator.py) can be sent. 

## Timeline
Since it might not always be possible to run the heavy [Isaac Sim](https://developer.nvidia.com/isaac/sim) simulation a timeline can be saved to be replayed later. The files in the [timelines](timelines) folder describe a timeline, which can be replayed when running [communicator.py](communicator.py) with configured timeline [settings](#settings).

> [!TIP]
> It is also possible to test the connection and send timeline data without setting up the [Isaac Sim](https://developer.nvidia.com/isaac/sim) simulation. You can do this by running the [communicator.py](communicator.py) file, after commenting out omni imports that get referenced and cause an error when running the [communicator.py](communicator.py) file.

## Situation
Besides outputting data on the connection the simulation can also utilize situation data as input to [setup the environment](environment_setup.py) in the simulation. The files in the [situations](situations) folder describe such a situation and can be generated by the [ISBEP-Server](https://github.com/marnikdenouden/ISBEP-Server). Simply add a new json file and update the situation [settings](#settings).

## Util
In order to make developments easier the [util.py](util.py) provides some functionality. It allows log messages to be more formatted to show their context and allows context to be defined to show or not. The util also has the performance_timestamp(label:str) method that can be called to log how long the section until the previous timestamp took to execute.

# Simulation Installation
The following steps can be taken to install the simulation locally.

- Ensure all local [dependencies](#dependencies) are met
- Create a folder with the [simulation folder name](#simulation-folder-name) in the `omni.isaac.examples.user_examples` folder of [Isaac Sim](https://developer.nvidia.com/isaac/sim).
- Clone [this]() repository into the newly created folder

#### Simulation Folder Name
    git_isaac_sim

# Simulation Usage
In order to run an installed simulation follow the next steps.

- Load the installed simulation project in [Isaac Sim](https://developer.nvidia.com/isaac/sim). 
- Click `Isaac Examples`, `User examples` and then `ISBEP Sim`.
- Press the `Load` button that appeared for the example.
- Press `Play` after the simulation has loaded.

# Dependencies
[Isaac Sim](https://developer.nvidia.com/isaac/sim) is required for running and developing this project. The simulation code uses [python](https://www.python.org/) and some additional modules, which could be installed using [pip](https://pypi.org/project/pip/). Even though python is already included in [Isaac Sim](https://developer.nvidia.com/isaac/sim) it might still need to be updated or have modules installed by following [Isaac sim python install](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html).

## Local Requirements
Download and install the following software.
- [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/index.html) (Includes python)
- [Python](https://www.python.org/downloads/) (For running without simulation)
