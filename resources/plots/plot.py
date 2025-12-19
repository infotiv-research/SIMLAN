import matplotlib.pyplot as plt
import pandas as pd

plt.style.use("seaborn-v0_8-whitegrid")

# Data: Camera Scalability
df1 = pd.DataFrame(
    {
        "Cameras": range(1, 13),
        "Image Only (20Hz)": [99, 56, 46, 34, 32, 23, 21, 19, 17, 15, 8, 7],
        "Complex Feed (Img+Depth+Sem)": [44, 22, 14, 9, 9, 7, 5, 4, 5, 3, 3, 2],
    }
)

plt.figure(figsize=(10, 6))


plt.plot(
    df1["Cameras"],
    df1["Image Only (20Hz)"],
    marker="o",
    linewidth=2,
    label="Image Only (20Hz)",
)
plt.plot(
    df1["Cameras"],
    df1["Complex Feed (Img+Depth+Sem)"],
    marker="s",
    linewidth=2,
    linestyle="--",
    label="Complex (Img+Depth+Sem)",
)

# plt.title('Impact of Camera Count & Feed Type', fontsize=14, weight='bold')
plt.xlabel("Number of Cameras", fontsize=12)
plt.ylabel("Real Time Factor (RTF) %", fontsize=12)
plt.xticks(range(1, 13))
plt.legend(fontsize=12)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("camera_impact_chart.png", dpi=300)
plt.show()
