#! /usr/bin/gnuplot

set terminal wx persist

ref_la="feet_follower_feet-follower-left-ankle.dat"
cor_la="feet_follower_correction-left-ankle.dat"

ref_ra="feet_follower_feet-follower-right-ankle.dat"
cor_ra="feet_follower_correction-right-ankle.dat"

ref_com="feet_follower_feet-follower-com.dat"
cor_com="feet_follower_correction-com.dat"

ref_zmp="feet_follower_feet-follower-zmp.dat"
cor_zmp="feet_follower_correction-zmp.dat"

set multiplot layout 2,2
plot ref_la u 5 w l, cor_la u 5 w l
plot ref_ra u 5 w l, cor_ra u 5 w l

plot ref_com u 2 w l, cor_com u 2 w l
plot ref_zmp u 2 w l, cor_zmp u 2 w l
unset multiplot
