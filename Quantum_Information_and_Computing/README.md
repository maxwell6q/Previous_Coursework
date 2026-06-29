# Quantum Information and Computing

This directory contains reports and accompanying MATLAB files for laboratory exercises and homework for the master's course on Quantum Information and Computing.


## Laboratory Exercises
During the course, two core experiments in quantum mechanics were conducted. 
This repository contains the data processing, statistical analysis, and theoretical modeling for the experiments.

### 1. CHSH Inequality Violation (Quantum Entanglement Validation)
The objective of this activity was to generate entabgled photon pairs through Spontaneous Parametric Down-Conversion using a lon-linear optical crystal, and measure them to experimentally violate the CHSH inequality, providing evidence against local hidden variable theories.

![Lab Setup](https://github.com/maxwell6q/Previous_Coursework/blob/main/Quantum_Information_and_Computing/Figures/LabSetup.jpg)


* **Data Preparation:** Processed raw timestamp data from sensors in Geiger mode, applying a $3\sigma$ time-window filtering method based on Gaussian curve fitting to accurately identify true photon coincidences amidst background noise.
* **Error Propagation:** Implemented a Gaussian error propagation model to estimate variance and confidence intervals, assuming Poissonian errors for sensor measurements.
* **Result:** Successfully violated the CHSH inequality ($\delta_{CHSH} = 2.5 > 2$) with a $1\sigma$ confidence interval of $\pm 0.01$.

### 2. Quantum State Tomography
The second activity focused on completely characterizing the quantum state (density matrix) of the entangled photon pairs by analyzing 16 distinct measurement projections.

* **Linear Inversion:** Initially reconstructed the density matrix using matrix inversion; however, vulnerability to measurement noise resulted in non-physical artifacts (negative eigenvalues).
* **Maximum Likelihood Estimation (MLE):** Applied maximum likelihood estimation to find the most likely strictly physical (positive semidefinite, Hermitian, and unit trace) density matrix.
* **State Validation:** Evaluated the reconstructed states using Fidelity, Concurrence, and Von-Neumann Entropy metrics, coupled with a statistical simulation utilizing 100 synthetic Poissonian datasets to confidently map statistical errors.


## Theoretical Homework
This section of the repository contains theoretical problem sets exploring the foundational mathematics of quantum mechanics, with a focus on system evolution, coupled states, and entanglement bounds.
* **Temporal Evolution and State Trajectories:** Modeled the continuous-time dynamics of a two-level system governed by a specific Hamiltonian. This involved computing eigenvalues to determine the system's energy gap, deriving the unitary time-evolution operator, and mapping the temporal evolution of the state as a rotational trajectory on the Bloch sphere.
* **Coupled Systems and Generalized Measurements:** Analyzed a bipartite system comprising a qubit interacting with an ancillary qutrit via a defined unitary transformation. The core focus was on isolating the primary subsystem's state using the partial trace operation and formulating Positive-Operator Valued Measures (POVMs) to compute generalized post-measurement states and outcome probabilities
* **Parametric Entanglement and CHSH Bounds:** Investigated a non-maximally entangled two-qubit state parameterized by an angle $\theta$. By calculating the expectation values of arbitrary spin observables defined by Pauli matrices, the precise mathematical boundary conditions required to violate the CHSH inequality were determined.