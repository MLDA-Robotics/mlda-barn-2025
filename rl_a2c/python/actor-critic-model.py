import torch
import torch.nn as nn
import torch.optim as optim

class LidarCNN(nn.Module):
    def __init__(self, lidar_dim, output_dim=32):
        super(LidarCNN, self).__init__()
        self.act_fea_cv1 = nn.Conv1d(
            in_channels=1, out_channels=output_dim, kernel_size=5, stride=2, padding=6) 
        self.act_fea_cv2 = nn.Conv1d(
            in_channels=output_dim, out_channels=output_dim, kernel_size=3, stride=2, padding=1)

    def forward(self, x):
        x = torch.relu(self.act_fea_cv1(x))
        x = torch.relu(self.act_fea_cv2(x))
        return x
            


class Actor(nn.Module):
    def __init__(self, lidar_dim, non_lidar_dim, action_dim):
        super(Actor, self).__init__()
        self.lidar_cnn = LidarCNN(lidar_dim, output_dim=32)
        
        with torch.no_grad():
            sample_input = torch.randn(1, 1, lidar_dim)
            sample_output = self.lidar_cnn(sample_input)
            conv_output_size = sample_output.view(1, -1).shape[1]

        self.fc1 = nn.Linear(conv_output_size, 64)
        self.fc2 = nn.Linear(64 + non_lidar_dim, 64)
        self.fc3 = nn.Linear(64, action_dim)

        torch.nn.init.xavier_uniform_(self.fc1.weight)
        torch.nn.init.xavier_uniform_(self.fc2.weight)

    def forward(self, lidar, non_lidar):
        x = torch.relu(self.lidar_cnn(lidar))
        x = torch.relu(self.fc1(x))
        x = torch.cat((x, non_lidar), dim=-1)
        x = torch.relu(self.fc2(x))
        return self.fc3(x)
    
class Critic(nn.Module):
    def __init__(self, lidar_dim, non_lidar_dim):
        super(Critic, self).__init__()
        self.lidar_cnn = LidarCNN(lidar_dim, output_dim=32)

        with torch.no_grad():
            sample_input = torch.randn(1, 1, lidar_dim)
            sample_output = self.lidar_cnn(sample_input)
            conv_output_size = sample_output.view(1, -1).shape[1]

        self.fc1 = nn.Linear(conv_output_size, 64)
        self.fc2 = nn.Linear(64 + non_lidar_dim, 64)
        self.fc3 = nn.Linear(64, 1)

        torch.nn.init.xavier_uniform_(self.fc1.weight)
        torch.nn.init.xavier_uniform_(self.fc2.weight)

    def forward(self, lidar, non_lidar):
        x = torch.relu(self.lidar_cnn(lidar))
        x = torch.cat((x, non_lidar), dim=-1)
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

if __name__ == "__main__":
    lidar_dim = 720
    non_lidar_dim = 1
    action_dim = 2
    actor = Actor(lidar_dim, non_lidar_dim, action_dim)
    critic = Critic(lidar_dim, non_lidar_dim)  
    
    print(actor)
    print(critic)
    
