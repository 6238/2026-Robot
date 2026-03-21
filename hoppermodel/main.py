import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import models, transforms
from PIL import Image
import os

# 1. Custom Dataset for Auto-Labeled Hopper Images
class HopperDataset(Dataset):
    def __init__(self, image_dir, transform=None):
        self.image_dir = image_dir
        self.transform = transform
        # Assumes filenames are "timestamp_0.45.jpg" where 0.45 is the fill %
        self.images = [f for f in os.listdir(image_dir) if f.endswith('.jpg')]

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_name = self.images[idx]
        img_path = os.path.join(self.image_dir, img_name)
        image = Image.open(img_path).convert('RGB')
        
        # Extract percentage from filename (e.g., "shot_0.75.jpg" -> 0.75)
        label = float(img_name.split('_')[-1].replace('.jpg', ''))
        label = torch.tensor(label, dtype=torch.float32)

        if self.transform:
            image = self.transform(image)
            
        return image, label

# 2. Model Definition
class HopperRegressionModel(nn.Module):
    def __init__(self):
        super(HopperRegressionModel, self).__init__()
        self.model = models.mobilenet_v3_small(weights='DEFAULT')
        in_features = self.model.classifier[3].in_features
        
        # Regression head: Linear -> Sigmoid (constrains output to 0.0-1.0)
        self.model.classifier[3] = nn.Sequential(
            nn.Linear(in_features, 1),
            nn.Sigmoid()
        )

    def forward(self, x):
        return self.model(x)

# 3. Training Setup
def train_hopper_model(data_path, epochs=20, batch_size=32, lr=1e-4):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # Augmentations to handle "Jostling" (Noise, Blur, Slight Rotations)
    train_transforms = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.RandomRotation(10), # Simulate robot tilt
        transforms.ColorJitter(brightness=0.2, contrast=0.2), # Field lighting
        transforms.GaussianBlur(kernel_size=(3, 3), sigma=(0.1, 2.0)), # Motion blur
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    dataset = HopperDataset(data_path, transform=train_transforms)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    model = HopperRegressionModel().to(device)
    optimizer = optim.Adam(model.parameters(), lr=lr)
    
    # Huber Loss: Robust against outliers from bouncing game pieces
    criterion = nn.HuberLoss(delta=0.1)

    print(f"Starting training on {device}...")
    for epoch in range(epochs):
        model.train()
        running_loss = 0.0
        
        for images, labels in dataloader:
            images, labels = images.to(device), labels.to(device)
            
            optimizer.zero_grad()
            outputs = model(images).squeeze()
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()
            
        print(f"Epoch [{epoch+1}/{epochs}], Loss: {running_loss/len(dataloader):.6f}")

    # Save the weights for deployment
    torch.save(model.state_dict(), "hopper_v1.pth")
    print("Training complete. Model saved.")

# 4. Run Training
if __name__ == "__main__":
    # Ensure you have a directory of images named like "frame_0.50.jpg"
    # train_hopper_model(data_path="path/to/your/dataset")
    pass