import os
import torch
import argparse
import numpy as np
from urllib.request import urlretrieve
import open3d as o3d
import MinkowskiEngine as ME
from datasets import SimulatorDataset
from model.minkunet import MinkUNet18A


parser = argparse.ArgumentParser()
parser.add_argument('--file_name', type=str, default='1.ply')
parser.add_argument('--weights', type=str, default='weights.pth')

VOXEL_SIZE = 0.05


def train():
    # loss and network
    criterion = nn.CrossEntropyLoss()
    net = MinkUNet14A(in_channels=3, out_channels=2, D=4)
    print(net)

    # a data loader must return a tuple of coords, features, and labels.
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    net = net.to(device)
    optimizer = SGD(net.parameters(), lr=1e-2)

    dataset = SimulatorDataset(voxel_size=VOXEL_SIZE)
    dataloader = torch.utils.data.DataLoader(dataset)

    for datapoint in dataloader:
        optimizer.zero_grad()

        # Get new data
        coords, feat, labels = datapoint
        input = ME.SparseTensor(feat, coords=coords).to(device)
        label = label.to(device)

        # Forward
        output = net(input)

        # Loss
        loss = criterion(output.F, label)
        print('Iteration: ', i, ', Loss: ', loss.item())

        # Gradient
        loss.backward()
        optimizer.step()

    # Saving and loading a network
    # torch.save(net.state_dict(), 'test.pth')
    # net.load_state_dict(torch.load('test.pth')


if __name__=="__main__":
    train()

