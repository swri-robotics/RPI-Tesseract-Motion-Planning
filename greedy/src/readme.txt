dependencies

	pip packages:

		scipy
		numpy
		robotraconteur (windows)
		RobotRaconteurCompanion
		general-robotics-toolbox
		pandas
		matplotlib


	ubuntu apt-packages
		RobotRaconteur
			sudo add-apt-repository ppa:robotraconteur/ppa
			sudo apt-get update

Instructions
	1. Make a copy of greedy/toolbox/robot_info/abb6640.yml and change it to new robot settings (POE).
	2. Under greedy/data/, run python cartesian2joint.py --file-name=*abb6640.yml*, switch to new robot yml. This script creates joint configuration of the full curve
	3. Under greedy/greedy_fitting, run python greedy_simplified.py --file-name=*abb6640.yml*, switch to new robot yml. 
	The three output files are command_backproj.csv, curve_fit_backproj.csv, and curve_fit_js.csv.