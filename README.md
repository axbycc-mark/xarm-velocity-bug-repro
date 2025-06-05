Copy the data files (.txt files) from any of the data*/ folders to your local system as well as the .cpp file.  

**times_us.txt**  is a list of integer timestamps in microseconds  
**qs.txt** and **qdots.txt** are a lists of desired joint angles corresponding with the times_us.txt.  

qdots.txt must have as many lines as times_us.txt. qs.txt must have either one line, or as many lines as times_us.txt. If qs.txt has only one line, it is used only for initializing the robot position and then pure velocity control will used during replay.  

Make sure that the Eigen dependency and the xArm C++ SDK are available on the system. Edit the .cpp file under the CONFIGURATION section appropriately.  
This should be enough to compile the .cpp file into an executable. Read the main() function of the executable for more details of the function.  

After running the executable, an actual_qdots.txt and commanded_qdots.txt will be generated. Using the .py file in this directory, you may generate the analysis plot. By default, all joint angles will appear in a single plot. You can use --separate-plots flag to change this behavior.  

Make sure numpy and matplotlib are available on your system. Here is what plot was generated on my machine. You can clearly observe spikes in the actual joint velocities (solid lines) vs the commanded joint velocities (dotted lines).


![replay_spike](https://github.com/axbycc-mark/xarm-velocity-bug-repro/assets/155058764/0193e9f2-476f-44aa-99ce-d4c5a01478e4)
