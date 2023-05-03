# Coordinated Evacuation
## Simulation
The repository assumes a simulation environment based on the environment proposed in the academic year 2021/2022.
The map can be generated using the repository [MapGen](https://github.com/marcope-98/MapGen).
The AppImage provides an sdf file that needs to be saved in the relative path: `./src/simulator/models/mindstorm_map/`.
After the file has been saved a prompt will be displayed containing the number of robots to spawn and their pose in the plane. Copy this information in the relative path: `./conf`.

To launch the simulation make sure that the bash script `./script.sh` has execution prilidges and that you have sourced ros2 in your workspace.
Then simply execute the script from the terminal.


