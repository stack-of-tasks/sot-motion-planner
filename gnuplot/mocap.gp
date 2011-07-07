#!/usr/bin/gnuplot

set terminal wx persist

error="feet_follower_error_estimator-error.dat"
waist="feet_follower_robot_dynamic-waist.dat"

plot error u 1:2 w l, waist u 1:2 w l
