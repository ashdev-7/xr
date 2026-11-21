import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams.update({"font.size": 12, "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 150})
# numbers measured by tools/baselines.py, tools/ablation.py and the oracle analysis (held-out flights)
fig, ax = plt.subplots(figsize=(9, 4.2))
names = ["Stand still", "PathGuard", "Oracle: all motion\n< 10 s known", "Oracle: all motion\n< 20 s known"]
vals = [3.51, 3.39, 3.39, 3.12]; cols = ["#8c959f", "#1f6feb", "#bbbbbb", "#bbbbbb"]
b = ax.bar(names, vals, color=cols)
for r, v in zip(b, vals): ax.text(r.get_x() + r.get_width()/2, v + .05, f"{v:.2f} m", ha="center")
ax.set_ylim(0, 4.3); ax.set_ylabel("ATE RMSE [m]")
ax.set_title("Held-out flights. Naive integration: 15,345 m (off the chart).", fontsize=11)
fig.tight_layout(); fig.savefig("figures/floor.png"); plt.close(fig)
fig, ax = plt.subplots(figsize=(7, 3.8))
names = ["Constant velocity", "Decaying velocity (1 s)", "Hold position (ours)"]; vals = [5.79, 3.83, 3.28]
b = ax.barh(names, vals, color=["#d1242f", "#f2a33a", "#1f6feb"])
for r, v in zip(b, vals): ax.text(v + .08, r.get_y() + r.get_height()/2, f"{v:.2f} m", va="center")
ax.set_xlabel("error over blind 5 s [m], held-out flights"); ax.set_xlim(0, 6.8)
fig.tight_layout(); fig.savefig("figures/blind.png"); plt.close(fig)
fig, ax = plt.subplots(figsize=(7, 3.8))
names = ["Full PathGuard", "Without motion prior"]; vals = [3.39, 154.63]
b = ax.barh(names, vals, color=["#1f6feb", "#d1242f"]); ax.set_xscale("log")
for r, v in zip(b, vals): ax.text(v * 1.1, r.get_y() + r.get_height()/2, f"{v:.2f} m", va="center")
ax.set_xlabel("ATE RMSE [m], log scale"); ax.set_xlim(1, 2000)
fig.tight_layout(); fig.savefig("figures/ablation_prior.png"); plt.close(fig)
