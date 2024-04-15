#!/bin/bash
# provide input bag, output bags prefix, and time fraction
echo $1, $2, $3
t0=`rosbag info -y -k start $1`
t1=`rosbag info -y -k end $1`
tfr=`echo "$t0 + ($t1 - $t0) * $3" | bc -l`
echo $t0, $t1, $tfr
rosbag filter $1 $2a.bag "t.secs <= $tfr"
rosbag filter $1 $2b.bag "t.secs > $tfr"
rosbag compress --lz4 $2a.bag
rosbag compress --lz4 $2b.bag
rm $2a.orig.bag
rm $2b.orig.bag
