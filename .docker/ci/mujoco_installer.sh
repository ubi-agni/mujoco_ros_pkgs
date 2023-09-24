#!/bin/bash

mkdir -p ${HOME}/mujoco
rm -rf ${HOME}/mujoco/*
echo parsing MuJoCo versions: $MUJOCO_VERSIONS
newstr=$(echo $MUJOCO_VERSIONS  | sed 's/\[//' ) # remove [
newstr2=$(echo $newstr | sed 's/\]//' ) # remove ]
IFS=, # split on ,
read -r -a versions <<< "$newstr2"
#set -o noglob # disable glob
#set -- $newstr2 # split+glob with glob disabled

for mj_version in "${versions[@]}"
do
	echo "installing MuJoCo Version ${mj_version} ..."
	available=$(curl -o /dev/null --silent -Iw '%{http_code}' \
		https://github.com/google-deepmind/mujoco/releases/download/${mj_version}/mujoco-${mj_version}-linux-x86_64.tar.gz)
	if [[ $available -eq 404 ]]; then
		echo "version ${mj_version} could not be found!"
		continue
	fi
	curl -O -L https://github.com/google-deepmind/mujoco/releases/download/${mj_version}/mujoco-${mj_version}-linux-x86_64.tar.gz || echo "could not find version ${mj_version}!"
	mkdir ${HOME}/mujoco/${mj_version}
	tar -xzf mujoco-${mj_version}-linux-x86_64.tar.gz -C ${HOME}/mujoco/${mj_version} --strip-components 1
	rm mujoco-${mj_version}-linux-x86_64.tar.gz
done
