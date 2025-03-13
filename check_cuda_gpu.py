import torch

# Check if CUDA is available
print(torch.cuda.is_available())

# Check the number of GPUs
print(torch.cuda.device_count())

