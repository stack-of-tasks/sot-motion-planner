#! /usr/bin/gnuplot

set terminal wx persist
#set terminal png

ref_la="feet_follower_feet-follower-left-ankle.dat"
cor_la="feet_follower_correction-left-ankle.dat"

ref_ra="feet_follower_feet-follower-right-ankle.dat"
cor_ra="feet_follower_correction-right-ankle.dat"

ref_com="feet_follower_feet-follower-com.dat"
cor_com="feet_follower_correction-com.dat"

ref_zmp="feet_follower_feet-follower-zmp.dat"
cor_zmp="feet_follower_correction-zmp.dat"

offset="feet_follower_correction-offset.dat"

set multiplot layout 2,2
plot ref_la u 1:5 w l title 'ref', \
     cor_la u 1:5 w l title 'cor', \
     offset u 1:2 w l title 'offset'
plot ref_ra u 1:5 w l, cor_ra u 1:5 w l

plot ref_com u 1:2 w l, cor_com u 1:2 w l
plot ref_zmp u 1:2 w l, cor_zmp u 1:2 w l
unset multiplot
