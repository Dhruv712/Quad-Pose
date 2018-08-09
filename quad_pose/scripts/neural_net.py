import rospy
import numpy as np

import torch
import torch.nn as nn
import pandas as pd
import torch.utils.data as data_utils

from torch.autograd import Variable
from sklearn.model_selection import train_test_split

from std_msgs.msg import Float32MultiArray

# Hyper-parameters 
input_size = 3
hidden_size = 320
output_size = 3
num_epochs = 100
batch_size = 15

##### FILENAME
csv = pd.read_csv('/home/msl/catkin_ws/src/quad_pose/pixel_pose.csv')

features = csv.drop(['xPose', 'yPose', 'zPose'], axis=1)
targets = csv.drop(['xPixel', 'yPixel', 'boxSize'], axis=1)

f_train, f_test, t_train, t_test = train_test_split(features, targets, test_size=0.25)

f_train = torch.from_numpy(np.asarray(f_train)).float()

f_test = torch.from_numpy(np.asarray(f_test)).float()

t_train = torch.from_numpy(np.asarray(t_train)).float()

t_test = torch.from_numpy(np.asarray(t_test)).float()

train = data_utils.TensorDataset(f_train, t_train)
train_loader = data_utils.DataLoader(train, batch_size=batch_size)

test = data_utils.TensorDataset(f_test, t_test)
test_loader = data_utils.DataLoader(test, batch_size=batch_size)

# Device Configuration
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class NeuralNet(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(NeuralNet, self).__init__()
        self.relu = nn.ReLU()
        self.input = nn.Linear(input_size, hidden_size) 
        self.hidden = nn.Linear(hidden_size, hidden_size)
        self.output = nn.Linear(hidden_size, output_size)
    
    def forward(self, x):
        out = self.input(x) # 1
        out = self.relu(out)
        out = self.hidden(out) # 2
        out = self.relu(out)
        out = self.hidden(out) # 3
        out = self.relu(out)
        out = self.hidden(out) # 4
        out = self.relu(out)
        out = self.hidden(out) # 5
        out = self.relu(out)
        out = self.output(out) # 6
        return out

model = NeuralNet(input_size, hidden_size, output_size).to(device)

# Loss and optimizer
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters())  

# Train the model
total_step = len(train_loader)
for epoch in range(num_epochs):
    for i, (pixels, pose_labels) in enumerate(train_loader):  
        # Move tensors to the configured device

        pixels = pixels.to(device)
        pose_labels = pose_labels.to(device)
        
        # Forward pass
        outputs = model(pixels)

        loss = criterion(outputs, pose_labels)
        
        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        if (i+1) % 108 == 0:
            print ('Epoch [{}/{}], Step [{}/{}], Loss: {:.4f}' 
                   .format(epoch+1, num_epochs, i+1, total_step, loss.item()))

# Test the model
# In test phase, we don't need to compute gradients (for memory efficiency)
with torch.no_grad():
    correct = 0
    total = 0
    for pixels, pose_labels in test_loader:
        pixels = pixels.to(device)
        pose_labels = pose_labels.to(device)
        outputs = model(pixels)
        predicted = outputs.data

    	predicted = predicted.numpy()
    	pose_labels = pose_labels.numpy()

        total += pose_labels.size / 3

    	row, col = pose_labels.shape

    	totalErrorX = 0
    	totalErrorY = 0
    	totalErrorZ = 0

    	for i in range(row):
    		innerLoop = 0
    		for j in range(col):
    			#print('Diff: {}'.format(predicted[i][j] - pose_labels[i][j]))
    			if (j == 0):
    				totalErrorX += predicted[i][j] - pose_labels[i][j]
    				if abs(predicted[i][j] - pose_labels[i][j]) < 0.1:
    					innerLoop += 1

    			elif (j == 1):
    				totalErrorY += predicted[i][j] - pose_labels[i][j]
    				if abs(predicted[i][j] - pose_labels[i][j]) < 0.1:
    					innerLoop += 1

    			elif (j == 2):
    				totalErrorZ += predicted[i][j]
    				if abs(predicted[i][j] - pose_labels[i][j] < 0.2):
    					innerLoop += 1


    			# if abs(predicted[i][j] - pose_labels[i][j])/pose_labels[i][j] < 0.1:
    			# 	innerLoop += 1
    		#print('Counter: {}'.format(innerLoop))
    		if innerLoop == 3:
    			correct += 1

    # print('Total X Difference: {} '.format(totalErrorX))
    # print('Total Y Difference: {} '.format(totalErrorY))
    # print('Total Z Difference: {}\n'.format(totalErrorZ))

    print('Accuracy of the network on 8,465 test pixels: {} %'.format(100 * correct / total))

# Save the model checkpoint
# torch.save(model.state_dict(), 'model.ckpt')