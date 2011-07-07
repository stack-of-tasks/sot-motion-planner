#!/bin/sh
RSYNC="rsync -avz --delete"

LAAS_USER=tmoulard

PROFILE_DIR=profiles/laas/install
PROFILE_DIR_STABLE=${PROFILE_DIR}/stable
PROFILE_DIR_UNSTABLE=${PROFILE_DIR}/unstable

$RSYNC ${HOME}/${PROFILE_DIR_UNSTABLE}/ \
    hrp2014c.laas.fr:${PROFILE_DIR_UNSTABLE}
$RSYNC ${HOME}/${PROFILE_DIR_STABLE}/ \
    hrp2014c.laas.fr:${PROFILE_DIR_STABLE}
$RSYNC /opt/grx3.0/HRP2LAAS/bin/StackOfTasks.so \
    hrp2014c.laas.fr:/opt/grx3.0/HRP2LAAS/bin/StackOfTasks.so.${LAAS_USER}
