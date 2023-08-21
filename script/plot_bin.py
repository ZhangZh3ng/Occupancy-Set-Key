import matplotlib.pyplot as plt
import numpy as np

def plot_colored_rings_and_sectors(num_rings, num_lines, ring_width=0.1):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

    # Generate theta values (angles) for lines
    theta = np.linspace(0, 2 * np.pi, num_lines, endpoint=False)

    # Define colors for sectors
    sector_colors = plt.cm.plasma(np.linspace(0, 1, num_lines))

    # Plot colored sectors
    # for i, t in enumerate(theta):
    #     ax.fill_between([t, t + 2 * np.pi / num_lines], 0, num_rings * ring_width, color=sector_colors[i])

    # Generate radii for rings
    radii = np.linspace(0, num_rings * ring_width, num_rings)

    # Define colors for rings
    ring_colors = plt.cm.viridis(np.linspace(0, 1, num_rings))

    # Plot colored rings
    # for r, color in zip(radii, ring_colors):
    #     circle = plt.Circle((0, 0), r, fill=False, color=color, linewidth=2)
    #     ax.add_artist(circle)

    # Set radial grid lines to be equally spaced
    # ax.set_rticks(radii)

    # Set azimuthal (angular) grid lines
    ax.set_xticks(theta)
    ax.set_yticklabels([])  # Hide radial labels (scale)
    ax.set_xticklabels([])  # Hide radial labels (scale)


    # Set the title
    # ax.set_title("Colored Rings and Sectors")

    plt.show()

# Specify the number of rings, number of lines, and ring width
num_rings = 12
num_lines = 30
ring_width = 0.1

# Call the function to plot the image with colored rings and sectors
plot_colored_rings_and_sectors(num_rings, num_lines, ring_width)
