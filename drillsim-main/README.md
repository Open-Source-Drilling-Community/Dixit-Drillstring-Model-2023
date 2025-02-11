# XOM Drill Sim

## Introduction

Welcome to the source code repository for ExxonMobil's transient drilling simulator library __drillsim__. This library enables to perform transient modeling of drill strings by taking the all the inputs in a excel sheet. This helps us in understanding the severity of vibrational dysfunctions on the drilling performance and assist in providing recommendations to various BUs.

## Model Description and assumptions
* DrillSim treats drill strings as lumped spring mass with certain stiffness.
* The drill-string is segregated into multiple elements comprising of drill-pipes, HWDP and collar elements. 
* Has 4 degrees of freedom per node (axial displacement , axial velocity , angular displacement and angular velocity).
* The axial displacement is one dimensional.
* Equations in Axial and rotational directions are coupled in friction model (stribeck friction model) which utilizes the resultant velocity to compute friction forces.
* well trajectory and buouyancy effects is taken into consideration when computing for resultant friction forces.
* Uniform drag due to mud is applied throughout the system. 
* Bit-rock interaction models is made using depth-of-cut and tracking revolution of bit-depth w.r.t hole-depth.
* Capable of modeling Mud-motor and Heave compensator dynamics.

___Note_:__ [Transient Drill string Modeling](https://wiki.na.xom.com/) wiki article provides details on governing equations and model set-up.

## Getting started

1. __Install GIT:__ In order to get started you will need to install Git on your local machine.  Git is free and open source and can be obtained by completing an EMIT software request.  __GIT for Windows__ (__Ver. 2.25.0__ or above) is available as of October 1, 2020.

2. __Install an IDE:__ You will also want to install an Integrated Development Environment (IDE) on your GME computer.  We highly encourage use of __Microsoft Visual Code__ (__VS Code__) as it is a light-weight open-source and highly extensible development environment.  VS Code adopts a very modular design.  Extensions may be installed easily as need.  In fact, VS Code will recommend extensions based on user actions.  You can obtain VS Code through a software request.

3. __Install Python:__ You will need to install interpreters for the programming languages you intend to use.  __Python 3.10.7__ is currently available upon completion of a software request. __Note_:__ Python packages may be obtained using [Nexus](https://goto/nexusdocs).  The ExxonMobil [Python](https://wiki.na.xom.com/index.php?title=Python&oldid=29489#Option_A) wiki article provides details on setting up user access to the Nexus repository.
4. __Install DrillSim:__ Finally you will need to install drillsim library using the following command:

       py -m pip install git+https://github.com/ExxonMobil/drillsim.git
       
   This automatically installs all the required libraries (numpy, pandas etc.,) in your system.
5. __Input excel sheet:__ Download the input excel sheet from this [link](https://mysite.na.xom.com/personal/upstreamaccts_anmanna/Documents/Shared%20with%20Everyone/EM_drill_sim_input_sheet.xlsx)

## Code requirements

After filling the excel sheet you will need to specify the excel sheet location and output folder location in the sample code named as scratch.ipynb in examples folder in this repository:
      ```
      from drillsim.driver import main_driver as main_driver
      
      # Enter output folder path
      outputFolderPath = r"D:\Project\output"
      
      # Enter Input excel sheet path
      input_excel_path = r"D:/Project/Input/EM_drill_sim_input_sheet.xlsx"
      
      main_driver(input_excel_path, outputFolderPath)
      ```
Upon completion of python program run you will observe two files in your output folder :
* __Outputs.html:__ This .html file consists of interactive plots of evolution of different drilling parameters.
* __Output.pickle:__ This .pickle file contains python dictionaries of all the raw data generated during simulation.
     


