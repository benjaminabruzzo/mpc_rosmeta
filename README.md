# For osqp:
	sudo apt install python-pip
	pip install osqp

# installation
	mkdir ~/ros/src/mpc_meta && cd ~/ros/src/mpc_meta && git init && git remote add gh git@github.com:westpoint-robotics/mpc_rosmeta.git && git pull gh master
	touch ~/ros/build_mpc.bash
	echo ". ~/ros/src/mpc_meta/scripts/build_mpc.bash" >> ~/ros/build_mpc.bash
	echo "source ~/ros/devel/setup.bash" >> ~/ros/build_mpc.bash
	cd ~/ros && chmod +x build_mpc.bash && bash build_mpc.bash

