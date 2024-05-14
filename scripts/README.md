Scripts Folder
This folder contains simulation scripts for the drone project. These scripts are used to model and test the drone dynamics and control algorithms before implementation on the actual hardware.

Drone Simulation
File: drone_simulation.py
This Python script simulates the translational dynamics of a ducted fan thrust vectoring drone. It uses numerical methods to solve the equations of motion and visualize the drone's behavior over time.

How to Run the Simulation
1. Ensure you have Python installed.
2. Install the necessary Python libraries:
        pip install numpy scipy matplotlib
3. Navigate to the scripts directory:
        cd your_project/scripts
4. Run the simulation script:
        python drone_simulation.py

Libraries Used
numpy: For numerical computations.
scipy: For solving differential equations.
matplotlib: For plotting the results.

Output
The script will generate a plot showing the drone's altitude over time.