Building Navigation Reinforcement Learning Project
==================================================

# Table of Contents
0. [Important Notes](#notes)
1. [Model Preparation](#prep)
2. [Training](#train)
3. [Playback](#play)

## Important Notes <a name="notes"></a>

(Notes that apply during development to keep track of unresolved issues, outstanding problems, or other relevant info)

1. The roboschool model of the stadium splits the geometry into three object files with one shared material file.  
2. The models have inertial models in the SDF but are static (find out why).
3. The lighting model in SDF is bit strange, see details here [Materials](#http://gazebosim.org/tutorials?tut=color_model&cat=)
4. Verify details of 12 ray intersection observation

Notes on SDF file format

[SDF File format in Gazebo](#https://www.youtube.com/watch?v=sHzC--X0zQE)

## Model Preparation <a name="prep"></a>

To run the model preparation, activate the conda environment:

- source activate sim

To run the test2 environment:
python simulator.py --render-image assets/output.png --export-object assets/output.obj --export-sdf assets/output.sdf --run-test test2 assets/test2.png

To run the building environment:
python simulator.py --render-image assets/output.png --export-object assets/output.obj --export-sdf assets/output.sdf --run-test building assets/building.png

### Preparing the input file (building plan)

The building model consists of several files and processes to extract the mesh from the building plan.  To prepare a
file for processing, start with a high resolution image of the building and trace lines into a separate layer.  Note
that the input file is essentially a bitmap because it processes only two colours, white (empty space), and non-white
(wall).  Use a single colour for the wall to ensure the wall colour is correctly detected by the algorithm.

The simulation.py program is a single entry point for executing all related algorithms and processes for the system in 
order to keep everything simple.  The building plan analysis algorithm extracts the walls from the model by using a
greedy algorithm to "grow" the walls as long as possible and split them under certain conditions.  Note the input file
has some restrictions.  The building must consist of an outside, connected wall consisting of only vertical or
horizontal lines.  No holes in the walls are possible.  All internal inaccessible spaces should also be connected walls 
separating the traverseable inside from the inaccessible interior.  Most types of rectilinear plan will work and to 
create interior walls, simply ensure that the wall is two pixels wide so that a closed loop can be formed.

Finally, a single line should be placed at the bottom of the plan.  This line is the scale and may represent any scale
desired, but will be used to calculate the appropriate scale for the building in the training model.

### Creating the OBJ Model

The first step is to create the OBJ file.  This file is in the [Wavefront OBJ File Format](#https://en.wikipedia.org/wiki/Wavefront_.obj_file).
The OBJ file also has an associated [Material File Format](#http://paulbourke.net/dataformats/mtl/).   This is used to
provide the walls and floors with a basic texture.

## Training <a name="train"></a>

## Playback <a name="play"></a>
