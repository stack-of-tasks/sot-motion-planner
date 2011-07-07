set terminal wx persist
a="/tmp/foo/feet_follower_feet-follower-com.dat"
b="/tmp/foo/feet_follower_feet-follower-zmp.dat"
c="/tmp/foo/hrp_sot_201104291639-astate.log"
plot a u ($1-8655):2 w l, b u ($1-8655):2 w l, c u 84 w l, c u 87 w l, c u ($84+$87) w l
