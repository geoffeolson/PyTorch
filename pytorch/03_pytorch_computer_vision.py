import torch
from torch import nn
import torchvision
from torchvision import datasets
from torchvision.transforms import ToTensor
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
from timeit import Timer
from tokenize import PlainToken
from helper_functions import accuracy_fn
from timeit import default_timer as timer  
import tqdm
from tqdm.auto import tqdm

class FashionMNIST_ModelV0(nn.Module):
    def __init__(self, input_shape: int, hidden_units: int, output_shape: int):
        super().__init__()
        self.layer_stack = nn.Sequential(
            nn.Flatten(),
            nn.Linear(in_features=input_shape, out_features=hidden_units),
            nn.Linear(in_features=hidden_units, out_features=output_shape)
        )

    def forward(self, x):
        return self.layer_stack(x)

class FashionMNIST_Training:
    """
    Object for training a FashionMNIST computer vision model
    """
    def __init__(self):
        self.seed = 42          # random number generator seed
        self.batch_size = 32    # number of images per batch
        self.epochs = 3         # number of epochs to train for

        self.train_data = datasets.FashionMNIST(
            root="data",
            train=True,
            download=True,
            transform=ToTensor(),
            target_transform=None
            )

        self.test_data = datasets.FashionMNIST(
            root="data",
            train=False,
            download=True,
            transform=ToTensor()
            )

        self.train_dataloader = DataLoader(
           self.train_data,
           batch_size= self.batch_size,
           shuffle=True
        )

        self.test_dataloader = DataLoader(
            self.test_data,
            batch_size=self.batch_size,
            shuffle=False
        )

        self.class_names = self.train_data.classes
        self.model_0 = FashionMNIST_ModelV0( 
            input_shape=784,
            hidden_units=10,
            output_shape=len(self.class_names)
        )
        self.model_0.to("cpu")
        self.loss_fn = nn.CrossEntropyLoss()
        self.optimizer = torch.optim.SGD(params=self.model_0.parameters(), lr=0.1)

        train_features_batch, train_labels_batch = next(iter(self.train_dataloader))
        random_idx = torch.randint(0, len(train_features_batch), size=[1]).item()
        img, label = train_features_batch[random_idx], train_labels_batch[random_idx]

    def review_dataset(self):
        """Review the data set"""
        image, label = self.train_data[0]
        print(f"image shape: {image.shape}, [color_channels=1, height=28, width=28]")
        print(f" length of training data: {len(self.train_data.data)},  length of training data targets: {len(self.train_data.targets)}")
        print(f" length of test data: {len( self.test_data.data)},  length of test data targets: {len( self.test_data.targets)}")
        print(f"class names: {self.class_names}")
        print(f"Train Data Loader: {self.train_dataloader}")
        print(f"Test Data Loaders: {self.test_dataloader}")
        print(f"Length of train dataloader: {len(self.train_dataloader)}")
        print(f"Length of test dataloader: {len(self.test_dataloader)}")
        # Plot images
        torch.manual_seed(self.seed)
        fig = plt.figure(figsize=(9, 9))
        rows, cols = 4, 4
        for i in range(1, rows * cols + 1):
            random_idx = torch.randint(0, len(self.train_data), size=[1]).item()
            img, label = self.train_data[random_idx]
            fig.add_subplot(rows, cols, i)
            plt.imshow(img.squeeze(), cmap="gray")
            plt.title(self.class_names[label])
            plt.axis(False)
        plt.show(block=True)


    def main_loop(self):
        """Main training loop for the FashionMNIST model"""
        torch.manual_seed(self.seed)
        train_time_start_on_cpu = timer()
        for epoch in tqdm (range(self.epochs)):

            # TRAINING
            print(f"Epoch: {epoch}\n----")
            train_loss = 0 
            for batch, (X,y) in enumerate(self.train_dataloader):
                self.model_0.train()
                y_pred = self.model_0(X)
                loss = self.loss_fn(y_pred, y)
                train_loss += loss
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

                if batch % 400 == 0:
                    print(f"Looked at {batch * len(X)}/{len(self.train_dataloader.dataset)} samples")
            train_loss /= len( self.train_dataloader)

            # TESTING
            test_loss, test_acc = 0, 0
            self.model_0.eval()
            with torch.inference_mode():
                for X, y in self.test_dataloader:
                    test_pred = self.model_0(X)
                    test_loss += self.loss_fn(test_pred, y) 
                    test_acc  += accuracy_fn(y_true = y, y_pred = test_pred.argmax(dim=1))
                test_loss /= len(self.test_dataloader)
                test_acc  /= len(self.test_dataloader)
            print(f"\nTrain loss: {train_loss:.5f} | Test loss: {test_loss:.5f}, Test acc: {test_acc:.2f}%\n")

        train_time_end_on_cpu = timer()
        total_train_time_model_0 = FashionMNIST_Training.print_train_time(
                                start = train_time_start_on_cpu,
                                end   = train_time_end_on_cpu,
                                device = str(next(self.model_0.parameters()).device)
                                )

    @staticmethod
    def print_train_time( start: float, end: float, device: torch.device = None):
        """Print the total trainning time"""
        total_time = end - start
        print (f"train time on {device}: {total_time:.3f} seconds")
        return total_time

training = FashionMNIST_Training()
training.review_dataset()
training.main_loop()
print("")

train_data = datasets.FashionMNIST(
    root="data",
    train=True,
    download=True,
    transform=ToTensor(),
    target_transform=None
    )

test_data = datasets.FashionMNIST(
    root="data",
    train=False,
    download=True,
    transform=ToTensor()
    )

BATCH_SIZE = 32 # number of images per batch

train_dataloader = DataLoader(
   train_data,
   batch_size= BATCH_SIZE,
   shuffle=True
)

test_dataloader = DataLoader(
    test_data,
    batch_size=BATCH_SIZE,
    shuffle=False
)

image, label = train_data[0]
class_names = train_data.classes
torch.manual_seed(42)
train_features_batch, train_labels_batch = next(iter(train_dataloader))
random_idx = torch.randint(0, len(train_features_batch), size=[1]).item()
img, label = train_features_batch[random_idx], train_labels_batch[random_idx]


torch.manual_seed(42)

model_0 = FashionMNIST_ModelV0( 
    input_shape=784,
    hidden_units=10,
    output_shape=len(class_names)
    )
model_0.to("cpu")

loss_fn = nn.CrossEntropyLoss()

optimizer = torch.optim.SGD(params=model_0.parameters(), lr=0.1)

def print_train_time( start: float, end: float, device: torch.device = None):
    total_time = end - start
    print (f"train time on {device}: {total_time:.3f} seconds")
    return total_time
    
# MAIN LOOP
torch.manual_seed(42)
train_time_start_on_cpu = timer()
epochs = 3
for epoch in tqdm (range(epochs)):

    # TRAINING
    print(f"Epoch: {epoch}\n----")
    train_loss = 0 
    for batch, (X,y) in enumerate(train_dataloader):
        model_0.train()
        y_pred = model_0(X)
        loss = loss_fn(y_pred, y)
        train_loss += loss
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if batch % 400 == 0:
            print(f"Looked at {batch * len(X)}/{len(train_dataloader.dataset)} samples")
    train_loss /= len( train_dataloader)

    # TESTING
    test_loss, test_acc = 0, 0
    model_0.eval()
    with torch.inference_mode():
        for X, y in test_dataloader:
            test_pred = model_0(X)
            test_loss += loss_fn(test_pred, y) 
            test_acc  += accuracy_fn(y_true = y, y_pred = test_pred.argmax(dim=1))
        test_loss /= len(test_dataloader)
        test_acc  /= len(test_dataloader)
    print(f"\nTrain loss: {train_loss:.5f} | Test loss: {test_loss:.5f}, Test acc: {test_acc:.2f}%\n")

train_time_end_on_cpu = timer()
total_train_time_model_0 = print_train_time(start = train_time_start_on_cpu,
                                            end = train_time_end_on_cpu,
                                            device = str(next(model_0.parameters()).device)
                                            )