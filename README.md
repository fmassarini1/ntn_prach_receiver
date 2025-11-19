# ntn_prach_receiver
This repository provides a framework for generation, simulation, and evaluation of 5G NR PRACH (Physical Random Access Channel) in Non-Terrestrial Network (NTN) environments.
The project compares two state-of-the-art PRACH receivers:

MATLAB nrPRACHDetect (5G Toolbox)

srsPRACHDetector (srsRAN-matlab C++ MEX) 

The goal is to evaluate detection probability, timing estimation accuracy, and robustness against satellite Doppler, under realistic NTN channel models and across multiple elevation angles.

*Dependencies*
MATLAB Toolboxes:
  - 5G Toolbox
  - Satellite Communications Toolbox
  - Communications Toolbox

srsRAN_matlab Components:
  - srsPRACHgenerator
  - srsPRACHdemodulator
  - srsMEX.phy.srsPRACHDetector
