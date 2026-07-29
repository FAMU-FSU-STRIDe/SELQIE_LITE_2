# Variables
float64[] timing # seconds
LegCommand[] commands

# Optional playback control.
#
# Defaults (all zero) reproduce the original behaviour exactly: play the
# trajectory once, anchored at the moment the message arrives. Producers that
# do not set these (e.g. swing_leg_node) are unaffected.

# Number of times to play the trajectory back-to-back. 0 or 1 plays it once.
# Repeating natively in leg_trajectory_publisher_node avoids republishing the
# message once per cycle, which would otherwise reset every leg's playback to
# the start of its stride on each repeat.
uint32 loops

# Cycle duration in seconds used to schedule repeats. Each repeat starts
# exactly one period after the previous one, so repeats do not accumulate
# drift. If <= 0, it is derived from the timing array.
float64 period

# ROS time (seconds) at which playback should start. Lets several legs share
# one common phase reference even though their messages arrive at slightly
# different times. If <= 0, playback is anchored at the arrival time instead.
float64 start_time
