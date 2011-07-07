#! /usr/bin/gnuplot

set terminal wx persist

#hrp_r="hrp_sot_201105031708-rstate.log"
#hrp_a="hrp_sot_201105031708-astate.log"

hrp_r="hrp_sot-rstate.log"
hrp_a="hrp_sot-astate.log"

robot_waist="feet_follower_robot_dynamic-waist.dat"
robot_com="feet_follower_robot_feature_ref_com-errorIN.dat"
robot_zmp="feet_follower_robot_device-zmp.dat"

com_e="feet_follower_robot_task_com-error.dat"
la_e="feet_follower_robot_task_left-ankle-error.dat"
ra_e="feet_follower_robot_task_right-ankle-error.dat"

ref_la="feet_follower_feet-follower-left-ankle.dat"
cor_la="feet_follower_correction-left-ankle.dat"

ref_ra="feet_follower_feet-follower-right-ankle.dat"
cor_ra="feet_follower_correction-right-ankle.dat"

ref_com="feet_follower_feet-follower-com.dat"
cor_com="feet_follower_correction-com.dat"

ref_zmp="feet_follower_feet-follower-zmp.dat"
cor_zmp="feet_follower_correction-zmp.dat"	

offset="feet_follower_correction-offset.dat"

waist="feet_follower_robot_dynamic-waist.dat"

error="feet_follower_error_estimator-error.dat"

set style line 1 lt 1 lw 1 pt 3 lc rgb "red"
set style line 2 lt 1 lw 1 pt 3 lc rgb "blue"
set style line 3 lt 1 lw 1 pt 3 lc rgb "green"

#plot hrp_a u 108 w l t 'waist (a)', hrp_a u 105 w l t 'zmp (a)', \
#     hrp_a u ($108+$105) w l t 'zmp W (a)'

set multiplot layout 3,1
plot hrp_r u 87 w l t 'waist (r)' ls 1, \
     hrp_r u 84 w l t 'zmp (r)' ls 2, \
     hrp_r u ($87+$84) w l t 'zmp W (r)' ls 3

plot [:][-1:1.5] \
     robot_com u 1:2 w l t 'com (sot)' ls 2, \
     robot_zmp u 1:2 w l t 'zmp (sot)' ls 3, \
     robot_waist u 1:5 w l t 'waist (sot)' ls 1

plot [:][-0.1:0.1] \
     error u 1:2 w l t 'error x' ls 1, \
     error u 1:3 w l t 'error y' ls 2, \
     error u 1:4 w l t 'error theta' ls 3

unset multiplot

#     robot_zmp u 1:5 w l t 'zmp (sot)',


#plot hrp_r u 100 w l t 'dt (r)'

#plot hrp_a u 160 w l, hrp_a u 163 w l, hrp_a u ($160+$163) w l t "zmp w", \
#     hrp_r u 84 w l, hrp_r u 87 w l, hrp_r u ($84+$87) w l t "zmp w", \
#     cor_zmp u 1:2 w l, waist u 1:2 w l, ref_com u 1:2 w l
