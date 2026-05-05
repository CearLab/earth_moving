import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def show_sample(sample, pred=None, title="sample"):
    x = sample["input"]
    y = sample["target"][0]
    g_t, s_t, s_next = x[0], x[1], x[2]
    p = pred[0] if pred is not None else np.zeros_like(y)
    err = np.abs(p - y)

    fig = make_subplots(
        rows=2,
        cols=3,
        subplot_titles=("G_t", "S_t", "S_{t+1,cmd}", "Target G_{t+1}", "Pred P_{t+1}", "|Pred-Target|"),
    )

    for i, z in enumerate([g_t, s_t, s_next, y, p, err]):
        r, c = i // 3 + 1, i % 3 + 1
        fig.add_trace(go.Heatmap(z=z, colorscale="Viridis", showscale=False), row=r, col=c)

    fig.update_layout(height=700, width=1000, title=title)
    fig.update_xaxes(visible=False)
    fig.update_yaxes(visible=False)
    fig.show()


def show_loss(history):
    fig = go.Figure()
    fig.add_trace(go.Scatter(y=history["train_loss"], mode="lines+markers", name="train"))
    fig.add_trace(go.Scatter(y=history["val_loss"], mode="lines+markers", name="val"))
    fig.update_layout(title="BCE loss", xaxis_title="epoch", yaxis_title="loss", width=700, height=420)
    fig.show()
