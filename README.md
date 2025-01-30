0:02: This is a demonstration of NGC rover simulation containerized in a docker. 
 0:11: So we start up the Docker. 
 0:13: , by a simple command and utilizing proper network settings, and this docker can run in any system,, with proper network connection available. 
 0:26: So this docker gets the,, gets the MATLAB system running,, in the MATLAB software, the entire NGC rover has been simulated. 
 0:38: So once it starts, it runs the first CPV where the rover goes up to 7 m and due to a gyroscope error,, instead of turning right, it is going in circles. 
 0:57: So now in order to simulate other kinds of CPVs as well as the normal rover operation, we can have a simple variable setting that can be changed to obtain different settings. 
 1:12: So, for example, this is just the CPV 7, but we could also in run a different CPV. 
 1:24: That basically stimulates the rover flip. 
 1:27: So this CPV, what it does is that it,, after reaching a 7 m distance, the rover is given a hard turn to the left, and that causes the rover to go beyond the critical velocity and the rover flips. 
 1:47: So this is a rover flip simulation. 
 1:50: And we can also simulate the normal operation of the rover,. 
 1:58: By just not giving any CPV input. 
 2:01: , this is a normal operation of the rover where the rover turns right. 
 2:13: So this is an example of a Docker simulation, which can be used in the context of the fire program. 
 2:22: Thank you very much. 
