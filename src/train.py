import os
#import torch
import argparse
import numpy as np
from urllib.request import urlretrieve
print("i")
import open3d as o3d
print("o3d")
import MinkowskiEngine as ME
print("mink")
from datasets import SimulatorDataset
from model.minkunet import MinkUNet18A
from torch import nn
import torch
print("Imported train")

parser = argparse.ArgumentParser()
parser.add_argument('--file_name', type=str, default='1.ply')
parser.add_argument('--weights', type=str, default='weights.pth')

VOXEL_SIZE = 0.005


def train():
    # loss and network
    criterion = nn.CrossEntropyLoss()
    net = MinkUNet18A(in_channels=3, out_channels=2, D=4)
    #print(net)

    # a data loader must return a tuple of coords, features, and labels.
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    net = net.to(device)
    optimizer = torch.optim.SGD(net.parameters(), lr=1e-3)

    for epoch in range(10):
        dataset = SimulatorDataset(voxel_size=VOXEL_SIZE)
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
    # torch.save(net.state_dict(), 'test.pth')
    # net.load_state_dict(torch.load('test.pth')


if __name__=="__main__":
    train()

