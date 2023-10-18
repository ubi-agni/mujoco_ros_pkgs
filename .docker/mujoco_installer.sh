#!/bin/bash

rm -rf "${HOME}/mujoco"

for mj_version in $MUJOCO_VERSIONS
do
	echo "Installing MuJoCo Version ${mj_version} ..."

	if curl -sSLfO https://github.com/google-deepmind/mujoco/releases/download/${mj_version}/mujoco-${mj_version}-linux-x86_64.tar.gz; then
		mkdir -p "${HOME}/mujoco/${mj_version}"
		tar -xzf mujoco-${mj_version}-linux-x86_64.tar.gz -C "${HOME}/mujoco/${mj_version}" --strip-components 1
		rm "mujoco-${mj_version}-linux-x86_64.tar.gz"
	fi
done
