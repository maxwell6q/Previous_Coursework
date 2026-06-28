# Parameter Estimation for SEIR Epidemic Models using MCMC
This repository contains the implementation of a parameter estimation problem for a non-linear dynamic system using MATLAB. 
The project applies the Markov Chain Monte Carlo (MCMC) method to estimate the underlying parameters of a Susceptible-Exposed-Infected-Recovered (SEIR) compartment model.
The specific case study evaluates the spread of SARS-COV-2 in India during June 2020. 

## Methodology
* **Dynamical System Modeling:** Implemented a continuous-time 4-state SEIR model using ordinary differential equations to model population transitions.
* **Measurement Model:** Generated synthetic noisy data for daily new infections using a Gamma distribution to account for unpredicted variances and unreported cases.
* **Estimation Algorithm:** Utilized the Metropolis-Hastings MCMC algorithm with Gaussian random walk proposals. 
* **Numerical Optimization:** Evaluated the log-posterior rather than the standard posterior to prevent numerical underflow and improve computational stability during iterations.

## Key Results
* Executed the MCMC algorithm over $10000$ iterations with a $2500$ sample burn-in period.
* Successfully achieved convergence for the incubation rate ($\alpha$) and effective contact rate ($\beta$), but not for the recovery rate ($\gamma$). The true posterior shows practically zero correlation between $\gamma$ and the other parameters.
* Reconstructed the predicted system dynamics, which aligned closely with the true simulated epidemic trajectory.

SEIR Dynamics True vs. Predicted             |  Parameter Traceplot
:-------------------------:|:-------------------------:
![SEIR True-Predicted](https://github.com/maxwell6q/Previous_Coursework/blob/main/Estimation_and_Filtering/Figures/seir.png) | ![Parameter Traceplots](https://github.com/maxwell6q/Previous_Coursework/blob/main/Estimation_and_Filtering/Figures/traceplots.png)
