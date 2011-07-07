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

waist="feet_follower_robot_dynamic-waist.dat"

set style line 1 lt 1 lw 1 pt 3 lc rgb "red"
set style line 2 lt 1 lw 1 pt 3 lc rgb "red"
set style line 3 lt 1 lw 1 pt 3 lc rgb "blue"
set style line 4 lt 1 lw 1 pt 3 lc rgb "blue"
set style line 5 lt 1 lw 1 pt 3 lc rgb "green"

set style line 6 lt 1 lw 1 pt 3 lc rgb "red"
set style line 7 lt 1 lw 1 pt 3 lc rgb "red"
set style line 8 lt 1 lw 1 pt 3 lc rgb "blue"
set style line 9 lt 1 lw 1 pt 3 lc rgb "blue"


plot cor_la u 5:9 w l ls 2, \
     cor_ra u 5:9 w l ls 4
