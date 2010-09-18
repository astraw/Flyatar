Scripts
*******

====================== ====================================================================
Script name            Description
====================== ====================================================================
open-loop-exp.py       the main experiment running script
convert_to_matfile.py  convert experimentally saved data to a .mat file for quick analysis
simple_plot_matfile.py a simple trajectory plotting script
====================== ====================================================================

Running an experiment
*********************

1) Start FView GUI and start the flytrax ROS plugin. Enable tracking.
2) Start the tracker ROS node (``rosrun flytrax tracker.py``)
3) Start the motor control stuff (``roslaunch stage_message_interface stage_message_interface.launch``)
4) Start the experiment control script (``python open-loop-exp.py``)
5) Follow the instructions by open-loop-exp.py to run ``rosbag record``

Converting experimental results to .mat file
********************************************

``python convert_to_matfile.py <OL_exp_infoDATE_TIME.txt> <OL_exp_rosDATE_TIME.bag> <outfile.mat>``

Plotting .mat file
******************

``python simple_plot_matfile.py <data.mat> <outfile.pdf>``
