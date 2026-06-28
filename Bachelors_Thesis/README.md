# Windmodeling and Control of Electric Urban Air Vehicles

Urban Air Mobility (UAM) is poised to revolutionize transportation.
However, operating in urban environments introduces significant challenges through turbulance and wind disturbances, emphasizing the need for responsive and robust flight control systems.
This project analyzes the dynamic model of the Bell Nexus 4EX Urban Air Taxi (UAT).
It incorporates realistic actuator dynamics and wind disturbances, and evaluates control strategies to maintain flight stability.

## Main Findings
* **Controller Evaluation**: Preexisting Proportional-Integral-Derivative (PID) controllers designed for idealized, reduced models failed when implemented in realistic models with actuator constraints, causing instabilities.
* **New Controller Design**: New controllers were designed using the root-locus method, explicitly considering actuator dynamics and implementation constraints.  
* **Simulation Performance**: The newly developed controllers effectively stabilized the aircraft and maintained the desired performance levels even under significant urban wind disturbances.
The figures show a comparison of the two controllers' responses to a step disturbance applied in form of longitudinal windspeed.
  
Preexisting             |  Newly Designed
:-------------------------:|:-------------------------:
![Preexisting PID, Disturbance Rejection - frontal wind](https://github.com/maxwell6q/Previous_Coursework/blob/main/Bachelors_Thesis/Figures/dis_rej_lon_pap.png) | ![Newly designed PID, Disturbance Rejection - frontal wind](https://github.com/maxwell6q/Previous_Coursework/blob/main/Bachelors_Thesis/Figures/dis_rej_lon.png)


* **Identified Limitations**: Both the preexisting and the new controller sets lacked thrust control.
In simulations with urban wind conditions, this absence led to altitude loss and poor passenger comfort ratings.
The figures below show the applied wind model in the time domain and the vertical groundspeed and acceleration of the UAT.
Future work must address thrust control to ensure passenger safety and flight stability.

Wind Model             |  Vertiacal Groundspeed and Acceleration
:-------------------------:|:-------------------------:
![Applied Wind Model in the Time Domain](https://github.com/maxwell6q/Previous_Coursework/blob/main/Bachelors_Thesis/Figures/dis_rej_urb_wind.png) | ![Vertiacl groundspeed and acceleration](https://github.com/maxwell6q/Previous_Coursework/blob/main/Bachelors_Thesis/Figures/dis_rej_urb_lon_w_cotra.png)
