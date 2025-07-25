import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 📁 Verzeichnisse
results_dir = "benchmark_results"
images_dir = "images"
os.makedirs(images_dir, exist_ok=True)

# 📥 Rohdaten einlesen
dfs = []
for fname in os.listdir(results_dir):
    if fname.endswith("_results.csv"):
        df = pd.read_csv(os.path.join(results_dir, fname))
        df["planner"] = fname.replace("_results.csv", "")
        dfs.append(df)

if not dfs:
    raise RuntimeError("❌ Keine Ergebnisdateien gefunden.")
data = pd.concat(dfs, ignore_index=True)

# 🔧 Einheitliche Planner-Farben & Reihenfolge
planner_order = [
    "BasicPRM",
    "Hierarchical-Basic",
    "Hierarchical-Lazy",
    "LazyPRM",
    "VisibilityPRM",
]
planner_colors = {
    "BasicPRM": "#1f77b4",
    "Hierarchical-Basic": "#ff7f0e",
    "Hierarchical-Lazy": "#2ca02c",
    "LazyPRM": "#d62728",
    "VisibilityPRM": "#9467bd"
}
fallback_colors = ["#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]

# 📏 Metriken & Layout
metrics = ["length", "euclidean_length", "time", "roadmap_size"]
metric_labels = {
    "length": "Pfadlänge",
    "euclidean_length": "Euklidische Länge",
    "time": "Laufzeit [s]",
    "roadmap_size": "Roadmap‑Größe",
}
scenes = data["scene"].unique()
n_scenes = len(scenes)
n_metrics = len(metrics)
row_height = 1.8
fig_height = row_height * n_scenes * n_metrics
fig_width = 14  # breiter für Texte rechts

fig, axes = plt.subplots(
    nrows=n_metrics,
    figsize=(fig_width, fig_height),
    sharey=False,
)

# 📊 Balken pro Metrik
for ax, metric in zip(axes, metrics):
    bar_height = 0.8 / len(planner_order)
    y_pos = np.arange(n_scenes)

    for i, planner in enumerate(planner_order):
        color = planner_colors.get(planner, fallback_colors[i % len(fallback_colors)])
        offsets = y_pos + (i - len(planner_order) / 2) * bar_height + bar_height / 2
        vals = []

        for scene in scenes:
            row = data[(data["scene"] == scene) & (data["planner"] == planner)]
            val = row[metric].values[0] if not row.empty else 0
            vals.append(val)

        bars = ax.barh(offsets, vals, height=bar_height, label=planner, color=color)

        # ✏️ Wert und Algorithmusname rechts anzeigen
        for bar, val, offset in zip(bars, vals, offsets):
            width = bar.get_width()
            xpos = width + (0.01 * max(vals) if max(vals) > 0 else 0.01)
            ax.text(
                xpos, offset,
                f"{val:.2f}",
                va='center', ha='left', fontsize=8, color='black'
            )
            ax.text(
                xpos + 0.1 * max(vals), offset,  # Abstand nach rechts
                planner,
                va='center', ha='left', fontsize=7, color='gray'
            )

    ax.set_yticks(y_pos)
    ax.set_yticklabels(scenes, fontsize=9)
    ax.set_xlabel(metric_labels[metric], fontsize=12)
    ax.set_title(f"{metric_labels[metric]}", fontsize=14, pad=8)
    ax.grid(axis="x", linestyle="--", alpha=0.4)

# 📌 Legende & Export
handles, labels = axes[0].get_legend_handles_labels()
fig.legend(
    handles, labels, title="Planner", fontsize=9, title_fontsize=10,
    loc="upper center", ncol=len(planner_order), frameon=True, bbox_to_anchor=(0.5, 1.02)
)

plt.tight_layout(rect=[0, 0, 1, 0.96])
outfile = os.path.join(images_dir, "benchmark_overview.png")
plt.savefig(outfile, dpi=300)
plt.close()
print(f"✅ Übersicht gespeichert unter: {outfile}")
