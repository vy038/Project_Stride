"""Train a CfC to imitate the joint's PID (behaviour cloning) and export it for the firmware.

    python train_cfc_controller.py log_*.csv            # writes into this bring-up project, then reflash

Inputs to the net (per 500 Hz step): err = target - angle, vel, sin(angle), cos(angle). sin/cos let it learn
gravity load, which depends on joint angle. Output: q-axis voltage Uq. The recurrent state is what lets it
reproduce the PID's integral term. Same cell as cfc-cortex-m's no-backbone config (ncps CfCCell, mode
"default", backbone_layers=0), so the firmware runs your verified cfc_step_nobackbone unchanged.

Writes:  lib/cfc/weights_nobackbone.h   (CFC_NB_* names, same layout as cfc-cortex-m's export_nobackbone.py)
         src/cfc_ctrl_io.h              (input normalization + linear readout, CFC_CTRL_TRAINED 1)
         src/cfc_golden_nb.h            (40 steps run through the trained cell, for 'cfc check' on the chip)
Needs:   pip install torch ncps==1.0.1 numpy
"""
import argparse, glob, os, sys
import numpy as np
import torch
from ncps.torch import CfCCell

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cfc_export import export   # noqa: E402

IN_DIM = 4

def flat(t): return t.detach().numpy().astype(np.float32).reshape(-1).tolist()

def load(paths):
    runs = []
    for p in paths:
        d = np.genfromtxt(p, delimiter=",", names=True)
        err, ang = d["target_rad"] - d["angle_rad"], d["angle_rad"]
        x = np.stack([err, d["vel_rads"], np.sin(ang), np.cos(ang)], 1).astype(np.float32)
        runs.append((x, d["uq_v"].astype(np.float32)))
    return runs

def windows(runs, L, stride):
    X, Y = [], []
    for x, y in runs:
        for s in range(0, len(x) - L + 1, stride):
            X.append(x[s:s + L]); Y.append(y[s:s + L])
    return np.stack(X), np.stack(Y)

def rollout(cell, head, x):
    h = torch.zeros(x.shape[0], cell.hidden_size)
    ys = []
    for t in range(x.shape[1]):
        o, h = cell(x[:, t], h, 1.0)        # ts = 1.0 per step, firmware does the same
        ys.append(head(o))
    return torch.cat(ys, 1)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--hidden", type=int, default=16)
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--window", type=int, default=200)
    ap.add_argument("--project", default=os.path.dirname(HERE))
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    paths = sorted({p for g in a.logs for p in glob.glob(g)})
    if len(paths) < 2: sys.exit("need at least 2 logs (one is held out for validation); 5+ is better")
    torch.manual_seed(a.seed); np.random.seed(a.seed)

    runs = load(paths)
    train_runs, val_runs = runs[:-1], runs[-1:]          # last log = held out, never trained on
    allx = np.concatenate([r[0] for r in train_runs]); ally = np.concatenate([r[1] for r in train_runs])
    in_mean, in_std = allx.mean(0), allx.std(0) + 1e-6
    out_mean, out_std = float(ally.mean()), float(ally.std() + 1e-6)
    norm = lambda rs: [((x - in_mean) / in_std, (y - out_mean) / out_std) for x, y in rs]
    Xtr, Ytr = windows(norm(train_runs), a.window, a.window // 4)
    Xva, Yva = windows(norm(val_runs), a.window, a.window)
    Xtr, Ytr, Xva, Yva = map(torch.from_numpy, (Xtr, Ytr, Xva, Yva))
    print(f"{len(paths)} logs, {len(Xtr)} train windows, {len(Xva)} val windows of {a.window} steps")

    cell = CfCCell(IN_DIM, a.hidden, mode="default", backbone_layers=0)
    head = torch.nn.Linear(a.hidden, 1)
    opt = torch.optim.Adam(list(cell.parameters()) + list(head.parameters()), lr=3e-3)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    for ep in range(a.epochs):
        cell.train(); perm = torch.randperm(len(Xtr))
        for i in range(0, len(Xtr), 32):
            b = perm[i:i + 32]
            loss = torch.mean((rollout(cell, head, Xtr[b]) - Ytr[b]) ** 2)
            opt.zero_grad(); loss.backward(); opt.step()
        sched.step()
        if ep % 50 == 0 or ep == a.epochs - 1:
            with torch.no_grad():
                v = torch.mean((rollout(cell, head, Xva) - Yva) ** 2).item()
            print(f"epoch {ep:4d}  train {loss.item():.4f}  val {v:.4f}  (normalized MSE, 1.0 = predicting the mean)")
    cell.eval()
    with torch.no_grad():
        pred = rollout(cell, head, Xva) * out_std + out_mean
        true = Yva * out_std + out_mean
        rmse = torch.sqrt(torch.mean((pred - true) ** 2)).item()
        r2 = 1 - torch.mean((pred - true) ** 2).item() / torch.var(true).item()
    print(f"held-out log: Uq RMSE {rmse:.3f} V, R^2 {r2:.3f}. Open-loop imitation only: closed-loop behaviour is the real test.")

    # ---- export ----
    gx = Xva[0, :40].numpy()
    with torch.no_grad():
        h = torch.zeros(1, a.hidden); outs = []
        for t in range(len(gx)):
            o, h = cell(torch.from_numpy(gx[t:t + 1]), h, 1.0); outs.append(o.squeeze(0).tolist())
    W = {nm: (layer.weight.detach().numpy().tolist(), layer.bias.detach().numpy().tolist())
         for nm, layer in [("FF1", cell.ff1), ("FF2", cell.ff2), ("TIME_A", cell.time_a), ("TIME_B", cell.time_b)]}
    lib, src = export(a.project, W, in_mean.tolist(), in_std.tolist(), flat(head.weight), head.bias.item(),
                      out_mean, out_std, gx.tolist(), outs,
                      f"{len(paths)} logs, held-out Uq RMSE {rmse:.3f} V, R^2 {r2:.3f}.")
    print(f"exported to {lib}/weights_nobackbone.h, {src}/cfc_ctrl_io.h, {src}/cfc_golden_nb.h")
    print("reflash, then: cfc check  ->  closed  ->  ctrl shadow  ->  ctrl cfc")


if __name__ == "__main__":
    main()
