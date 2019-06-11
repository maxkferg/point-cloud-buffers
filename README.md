# Dynamic Point Cloud Buffers

Dynamic Point Cloud Models with Point Cloud Buffers. This repository contains the code for training a sparse high-dimensional convolutional neural network for predicting valid/invalid points in a large point cloud. The sparse neural network is written using Pytorch and [Minkowski Engine](https://github.com/StanfordVL/MinkowskiEngine).

![Dynamic Point Cloud Buffer](/docs/animation.gif?raw=true "Point Cloud Buffer")

## Setup
Setup the Conda environment:
```sh
conda env create -f environment.yml
```
Next build Minkowski Engine using the instructions in the docs: [Minkowski Engine](https://github.com/StanfordVL/MinkowskiEngine).

## Training 
The network can be trained on any GPU with at least 8Gb of VRAM. For best results, we recommend training on at least 2 GPUs and increasing the batch_size to at least 16. To train:

```sh
python train.py --dataset=simulator/dataset/points2.ply' --voxel_size=0.005```
```

To fine-tune:

```sh
python train.py --dataset=simulator/dataset/points2.ply' --voxel_size=0.005 --weights=model.pth```
```

## Visualization
Videos of the raw dataset and trained models can be generated with `visualize.py`

```sh
python visualize.py```
```

## Synthetic Datasets
Synthetic datasets are created by running our autonomous turtlebot simulator and recording the resulting pointclouds. Datasets are stored in the src/simulator/dataset/ directory. The synthetic dataset is loaded into Pytorch by the `datasets.TD3Loader`.


## Exporting the environment
Setup the Conda environment:
```sh
conda env create -f environment.yml
```

## License
MIT