RoseLap: Laptime Simulation for FSAE

RoseLap is a laptime simulation package that includes the following:
- .DXF track loading
- Point-mass and two-tire physics models
	- Engine curve acceleration
	- Shifting time (power cuts)
	- Braking algorithm (with brake bias in two-tire model)
	- Aerodynamic effects
- 2D and 3D studies (plotting)
	- Detail views
	- Points simulation
- A brief command-line interface

-- INSTALLATION & PREREQUISITES --

Download the release .zip and move it to a working directory.

RoseLap is built for Python 2.7. You will need that installed along with the following packages (at command line, run `pip install <package-name>` or `python -m pip install <package-name>`):
- numpy
- ruamel.yaml
- docopt
- matplotlib

-- SETTING UP STUDIES --

Vehicles are built using YAML markup language. Refer to Vehicles/example_vehicle.yaml as a guide for vehicle parameters. Use a text editor to create a new vehicle in this folder and modify it to your needs.

Studies are built using YAML markup language. Refer to Studies/example_study.yaml as a guide for studies. Use a text editor to create new study in this folder and modify it to your needs.

After setting up your study, open up a command line and navigate to the RoseLap folder. From here, you can use the RoseLap CLI. Type `python roselap.py -h` to learn how to use the CLI to run and load studies. Results of studies will be stored in the Results folder and you can rename them for version control.
