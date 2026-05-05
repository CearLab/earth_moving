import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from tqdm.auto import tqdm


class SpillageDataset(Dataset):
    def __init__(self, samples):
        self.samples = samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        s = self.samples[idx]
        x = torch.from_numpy(s["input"]).float()
        y = torch.from_numpy(s["target"]).float()
        return x, y


class TinySpillageCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 32, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 16, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 1, 1),
            nn.Sigmoid(),
        )

    def forward(self, x):
        return self.net(x)


def split_samples(samples, val_ratio=0.2, seed=0):
    rng = np.random.default_rng(seed)
    idx = np.arange(len(samples))
    rng.shuffle(idx)
    n_val = int(round(val_ratio * len(samples)))
    val_idx = idx[:n_val]
    train_idx = idx[n_val:]
    train = [samples[i] for i in train_idx]
    val = [samples[i] for i in val_idx]
    return train, val


def train_model(train_samples, val_samples, epochs=20, batch_size=32, lr=1e-3, device=None):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    model = TinySpillageCNN().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.BCELoss()

    train_loader = DataLoader(SpillageDataset(train_samples), batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(SpillageDataset(val_samples), batch_size=batch_size, shuffle=False)

    hist = {"train_loss": [], "val_loss": []}

    for _ in tqdm(range(epochs), desc="epochs"):
        model.train()
        train_loss = 0.0
        n_train = 0
        for x, y in train_loader:
            x, y = x.to(device), y.to(device)
            opt.zero_grad(set_to_none=True)
            pred = model(x)
            loss = loss_fn(pred, y)
            loss.backward()
            opt.step()
            train_loss += loss.item() * x.size(0)
            n_train += x.size(0)

        model.eval()
        val_loss = 0.0
        n_val = 0
        with torch.no_grad():
            for x, y in val_loader:
                x, y = x.to(device), y.to(device)
                pred = model(x)
                loss = loss_fn(pred, y)
                val_loss += loss.item() * x.size(0)
                n_val += x.size(0)

        hist["train_loss"].append(train_loss / max(1, n_train))
        hist["val_loss"].append(val_loss / max(1, n_val))

    return model, hist, device
