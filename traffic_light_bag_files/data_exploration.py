import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

labels_1 = np.loadtxt("just_labels.txt")
labels_2 = np.loadtxt("loop_labels.txt")
labels = np.vstack([labels_1, labels_2])

df = pd.DataFrame(labels_2, columns=["img_name", "Traffic light colour"])

# Plot the distribution of the traffic light images
sns.set_style("white")

ax = sns.factorplot(x="Traffic light colour", data=df, kind="count")
ax.set_xticklabels(["none", "red", "yellow", "green"])

plt.draw()
plt.show()
