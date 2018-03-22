#!/bin/bash

src_dir="/cs-share/pradalier/vrep_ros_ws/src/"

find $src_dir -mtime -2 -type f 2>/dev/null 1>to_update.txt

#cat to_update.txt

while read line; do
	localFile=${line#*/src/}
	if [[ -f $localFile ]] ; then
		if [[ $line -ot $localFile ]]; then
			echo "$localFile already exist, you may want to merge it"
		fi
	else
		localDir=${localFile%/*}
		if [[ -d $localDir ]]; then
			cp "$line" "$localFile"
		else
			echo "dir $localDir do not exist, do not copy"
		fi
	fi
done < to_update.txt
