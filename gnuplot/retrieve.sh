#!/bin/sh
SCP=scp
LAAS_USER=tmoulard

XP_DIR=$HOME/exp/`date '+%d-%m-%Y_%Hh%M'`

mkdir $XP_DIR
scp $LAAS_USER@hrp2014c.laas.fr:'/tmp/feet*dat' $XP_DIR/
scp $LAAS_USER@hrp2014c.laas.fr:'/tmp/hrp_sot*log' $XP_DIR/
scp $LAAS_USER@hrp2014c.laas.fr:'/opt/grx3.0/HRP2LAAS/log/hrpsys.log' $XP_DIR/
