#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd edition, John Wiley & Sons, Chichester, UK. 
URL: https://www.fossen.biz/wiley  
    
Author:     Thor I. Fossen
"""
import os
import sys 
# import webbrowser
import matplotlib.pyplot as plt
import numpy as np
from python_vehicle_simulator.vehicles import shipClarke83 # (DSRV, frigate, otter, ROVzefakkel, semisub, shipClarke83, supply, tanker, remus100, torpedo)
from python_vehicle_simulator.lib import (
    printSimInfo, printVehicleinfo, simulate, plotVehicleStates, plotControls) #, plot3D)
from python_vehicle_simulator.lib.gnc import attitudeEuler

### Simulation parameters ###
sampleTime = 0.1                   # sample time [seconds]
T_acc = 600.0                         # acceleration time to full speed [seconds]
Tsim = 3*T_acc                       # total simulation time [seconds]
N = int(Tsim/sampleTime)                     # number of samples

# 3D plot and animation settings where browser = {firefox,chrome,safari,etc.}
numDataPoints = 50                  # number of 3D data points
FPS = 10                            # frames per second (animated GIF)
filename = '3D_animation.gif'       # data file for animated GIF
browser = 'safari'                  # browser for visualization of animated GIF

### Main program ###
def main():
    # vehicle = shipClarke83('headingAutopilot', -20.0, 70, 8, 6, 0.7, 0.5, 10.0, 1e5)

    # Commands from Scenario
    psi_desired = -80.0  # desired delta heading (deg)
    U_current = 10.0   # current speed (m/s)
    U_desired = 28.0   # desired speed (m/s)
    eta_init = np.array([1000, 1000, 0, 0, 0, np.pi/3], float)  # initial position and attitude

    # Main simulation loop 
    #[simTime, simData] = simulate(N, sampleTime, shipClarke83('headingAutopilot', psi_desired, 70, 8, 6, 0.7, 0.5, 10.0, 1e5))
    vehicle = shipClarke83('headingAutopilot', psi_desired, 70, 8, 6, 0.7, 0, 0, U_current, U_desired, True)

    DOF = 6                     # degrees of freedom
    t = 0                       # initial simulation time

    # Initial state vectors
    eta = eta_init                                   # position/attitude, user editable
    nu = vehicle.nu                              # velocity, defined by vehicle class
    u_actual = vehicle.u_actual                  # actual inputs, defined by vehicle class
    
    # Initialization of table used to store the simulation data
    simData = np.empty( [0, 2*DOF + 2 * vehicle.dimU], float)

    # guidance + path following 
    L = vehicle.L
    

    # Simulator for-loop
    for i in range(0,N+1):
        
        t = i * sampleTime      # simulation time
        
        # Vehicle specific control systems
        u_control = vehicle.headingAutopilot(eta,nu,sampleTime)   

     
        
        # Store simulation data in simData
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )
        simData = np.vstack( [simData, signals] ) 

        # Propagate vehicle and attitude dynamics
        [nu, u_actual]  = vehicle.dynamics(eta,nu,u_actual,u_control,sampleTime)
        eta = attitudeEuler(eta,nu,sampleTime)

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]
    [simTime, simData] = simulate(N, sampleTime, vehicle, eta_init)

    # 3D plots and animation
    plotVehicleStates(simTime, simData, 1)                    
    #plotControls(simTime, simData, shipClarke83('headingAutopilot', psi_desired, 70, 8, 6, 0.7, 0.5, 10.0, 1e5), 2)
    # plot3D(simData, numDataPoints, FPS, filename, 3)   
    
    """ Uncomment the line below for 3D animation in the web browswer. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))
    
    plt.show()
    # plt.close()

if __name__ == "__main__":

        # --- Định nghĩa Đầu vào (Inputs) ---
    
    # 1. Thông số tàu
    L_input = 80.0
    V_input = 15.0

    main()