import os
import argparse
import numpy as np
from urllib.request import urlretrieve
import open3d as o3d
import MinkowskiEngine as ME
from datasets import SimulatorDataset
from model.minkunet import MinkUNet18A
from torch import nn
import torch
print("Imported train modules")

parser = argparse.ArgumentParser()
parser.add_argument('--dataset', type=str, default='simulator/dataset/points2.ply')
parser.add_argument('--weights', type=str, default='weights.pth')
parser.add_argument('--voxel_size', type=float, default=0.005)


def train(model_class, voxel_size, dataset_path):
    # loss and network
    criterion = nn.CrossEntropyLoss()
    net = model_class(in_channels=3, out_channels=2, D=4)

    # a data loader must return a tuple of coords, features, and labels.
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    net = net.to(device)
    optimizer = torch.optim.SGD(net.parameters(), lr=1e-3)

    for epoch in range(10):
        dataset = SimulatorDataset(dataset_path, voxel_size=voxel_size)
        dataloader = torch.utils.data.DataLoader(dataset)
    
        for i,datapoint in enumerate(dataloader):
            optimizer.zero_grad()

            # Get new data
            coords, feat, labels = datapoint
        
            # Dont bother training on small datapoints
            if list(coords.size())[1]<10:
                print("Small Tensor")
                continue

            #print('Sparse Tensor:',coords, feat, labels)
            coords, feat = ME.utils.sparse_collate(coords, feat)
            inputs = ME.SparseTensor(feat, coords=coords).to(device)
            labels = labels.to(device)

            # Forward
            output = net(inputs)

            # Loss
            labels = torch.squeeze(labels).long()
            logits = output.F
            loss = criterion(logits, labels)
            print('Iteration: ', i, ', Loss: ', loss.item())

            # Gradient
            loss.backward()
            optimizer.step()

    # Saving and loading a network
    torch.save(net.state_dict(), 'model.pth')
    net.load_state_dict(torch.load('model.pth')


if __name__=="__main__":
    model_class = MinkUNet18A
    args = parser.parse_args()
    train(model_class, args.voxel_size, args.dataset_path)

