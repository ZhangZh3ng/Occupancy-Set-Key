import numpy as np
import matplotlib.pyplot as plt

def banding_lsh_probability(s, l, b):
    return 1 - (1 - s ** l) ** b

# Parameters for the curves
params = [
    {"l": 2, "b": 10},
    {"l": 6, "b": 10},
    {"l": 6, "b": 30},
    {"l": 5, "b": 16},
    {"l": 8, "b": 8},
    # {"l": 20, "b": 40},
    # {"l": 10, "b": 40},
    # {"l": 10, "b": 100},
]

# Generate a range of similarity scores
scores = np.linspace(0, 1, 100)

# Plotting
plt.figure(figsize=(10, 6))

for param in params:
    l = param["l"]
    b = param["b"]
    
    probabilities = [banding_lsh_probability(s, l, b) for s in scores]
    label = "l=" +str(l) +", b=" + str(b)
    plt.plot(scores, probabilities, label=label, linewidth=4)

# plt.title("Probability of sharing at least 1 same hash value")
plt.xlabel("Jaccard similarity", fontsize=16, fontweight='bold')
plt.ylabel("Hit Probability", fontsize=16, fontweight='bold')
plt.tick_params(axis='both', which='major', labelsize=18)
# plt.grid()
plt.legend(fontsize=16)
plt.show()
