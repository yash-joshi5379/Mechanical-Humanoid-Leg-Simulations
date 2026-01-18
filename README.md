# Mechanical-Humanoid-Leg-Simulations

This repository contains a modular MATLAB simulation suite for a multi-jointed robotic leg. The project focuses on the kinematic modelling, 3D visualization, and design optimization of three primary joint assemblies: a 3-DOF Coaxial Spherical Parallel Manipulator (Hip), a 1-DoF Crossed Four-Bar Linkage (Knee), and a 1-DoF Worm-Driven Four-Bar Linkage (Ankle).

## 🛠️ Setup 
1. To run these simulations, ensure you have MATLAB Optimization Toolbox (required for fsolve in the SPM calculations).

2. Clone this repository or download all source files.

3. Ensure Gear_m2_n24.stl is in the same directory as the scripts to allow the ankle gear to render.

4. Open MATLAB and run the desired script from the Command Window or Editor.

## 🚀 Usage

1. To simulate the entire integrated leg system, enter ```run('Leg.m')``` in the MATLAB Command Window or press the green Run button in the MATLAB Editor.
2. To simulate the hip joint, enter ```run('SPM.m')``` in the MATLAB Command Window or press the green Run button in the MATLAB Editor. 
3. To simulate the knee joint, enter ```run('CrossedFourBar.m')``` in the MATLAB Command Window or press the green Run button in the MATLAB Editor.
4. To simulate the ankle joint, enter ```run('WormDrive.m')``` in the MATLAB Command Window or press the green Run button in the MATLAB Editor. 

