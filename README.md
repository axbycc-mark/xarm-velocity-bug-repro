Copy the data files (.txt files) to your local system as well as the .cpp file.  
Make sure that the Eigen dependency and the xArm C++ SDK are available on the system. Edit the .cpp file under the CONFIGURATION section appropriately.  
This should be enough to compile the .cpp file into an executable. Read the main() function of the executable for more details of the function.  

After running the executable, an actual_qdots.txt and commanded_qdots.txt will be generated. Using the .py file in this directory, you may generate the analysis plot.
Make sure numpy and matplotlib are available on your system. Here is what plot was generated on my machine. You can clearly observe spikes in the actual joint velocities (solid lines) vs the commanded joint velocities (dotted lines).

![replay_spike](https://github.com/axbycc-mark/xarm-velocity-bug-repro/assets/155058764/0193e9f2-476f-44aa-99ce-d4c5a01478e4)
