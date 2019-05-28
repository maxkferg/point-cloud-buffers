# Sketchup Instructions

1) Export model
* Change to z axis default
* Export single face only

2) Place new file `y2e2.obj` in the source folder

2) Run `make_pybullet_model.py` to process the obj
```sh
# Makes a new file in the pybullet directory
python make_pybullet_model.py

# Copy the material file over as well
cp source/y2e2.mtl pybullet/y2e2.mtl 
```

3) Export obj
```
./obj2sdf --fileName="pybullet/y2e2.obj" --mergeMaterials=True
```

4) Edit materials
Correct all the material paths in the `pybullet/y2e2.mtl` file


5) Find the floor object in the SDF file.
Set the scale of the visual object to [0,0,0] to hide the floor
Add the following collision plane (Not needed anymore)	
```XML	
	<collision name='collision_1'>	
		<geometry>	
	        <plane>	
	          <normal>0 0 1</normal>	
	          <size>100 100</size>	
	        </plane>	
		</geometry>	
	</collision>
```