import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from matplotlib.gridspec import GridSpec
from RCAIDE.Framework.Core import Units

plt.style.use('seaborn-v0_8-whitegrid')

payload_range = {
    "Airport Planning Manual": {
        "range": [0.0, 441.388, 1674.949, 1935.783],
        "payload": (np.array([7100, 7100, 4200, 0.0]) / Units.lbs).tolist(),
        "payload + oew": (np.array([138000, 138000, 115000, 91500])).tolist(),
   },
    "ATR72-600": {
        "range": (np.array([   0.    , 343.69 , 1541.77 , 1963.79]) ).tolist(),
        "payload": (np.array([7100.00, 7100.00 , 4863.46, 0.0]) / Units.lbs).tolist(),
        "payload + oew": (np.array([61876.74, 61876.74, 52007.17,  40876.74]) / Units.lbs).tolist(),
   }

}

# Setup colors with a modern color palette
colors = ['#ff7f0e', '#1f77b4','#2ca02c', '#d62728']

# Create figure with GridSpec for better control over subplot sizing
fig = plt.figure(figsize=(16, 8))
gs = GridSpec(1, 1, width_ratios=[1], wspace=0.15)

# Create subplots
ax1 = fig.add_subplot(gs[0])
# ax2 = fig.add_subplot(gs[1])  # Independent y-axis for second plot

# Subplot 1: Payload vs Range
for i, (label, data) in enumerate(payload_range.items()):
    ax1.plot(data["range"], data["payload"],
            label=label,
            linewidth=2.5,
            marker='o',
            markersize=6,
            color=colors[i % len(colors)])
    
    # Add shaded area with better opacity
    ax1.fill_between(data["range"], data["payload"], alpha=0.15, color=colors[i % len(colors)])

# # Add annotations for key points (optional)
# for label, data in payload_range.items():
#     # Annotate maximum payload point
#     max_payload_idx = np.argmax(data["payload"])
#     ax1.annotate(f"Max: {data['payload'][max_payload_idx]:.0f} kg", 
#                 xy=(data["range"][max_payload_idx], data["payload"][max_payload_idx]),
#                 xytext=(10, 10), textcoords="offset points",
#                 fontsize=9, color=colors[list(payload_range.keys()).index(label) % len(colors)],
#                 bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.7))

# Subplot 2: Payload + OEW vs Range
# for i, (label, data) in enumerate(payload_range.items()):
#     ax2.plot(data["range"], data["payload + oew"],
#             label=label,
#             linewidth=2.5,
#             marker='o',
#             markersize=6,
#             color=colors[i % len(colors)])
    
#     # Add shaded area with better opacity
#     ax2.fill_between(data["range"], data["payload + oew"], alpha=0.15, color=colors[i % len(colors)])

# Configure subplot 1
ax1.set_ylabel('Payload (lbs)', fontsize=12, fontweight='bold')
ax1.set_xlabel('Range (nmi)', fontsize=12, fontweight='bold')
ax1.set_title('Payload vs Range', fontsize=14, fontweight='bold', pad=15)
ax1.grid(True, linestyle='--', alpha=0.7)
ax1.legend(frameon=True, fontsize=10, loc='best', 
          bbox_to_anchor=(0.5, -0.15), ncol=2, 
          framealpha=0.8, edgecolor='gray')

# Configure subplot 2
# ax2.set_ylabel('Payload + OEW (lbs)', fontsize=12, fontweight='bold')
# ax2.set_xlabel('Range (nmi)', fontsize=12, fontweight='bold')
# ax2.set_title('Operating Weight vs Range', fontsize=14, fontweight='bold', pad=15)
# ax2.grid(True, linestyle='--', alpha=0.7)

# Format ticks and axis limits
for ax in [ax1]: #ax2]:
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f'{x:,.0f}'))
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f'{x:,.0f}'))
    ax.tick_params(axis='both', which='major', labelsize=10)
    
    # Add thousands separator to x-axis
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f'{x:,.0f}'))
    
    # Add box around the plot
    ax.spines['top'].set_visible(True)
    ax.spines['right'].set_visible(True)
    ax.spines['bottom'].set_visible(True)
    ax.spines['left'].set_visible(True)
    
    for spine in ax.spines.values():
        spine.set_color('gray')
        spine.set_linewidth(0.8)

# Set appropriate limits
ax1.set_xlim(-200, 12000)
ax1.set_ylim(0, 100000)  # Adjusted to better show data
# ax2.set_xlim(-200, 12000)
# ax2.set_ylim(260000, 380000)  # Adjusted to better show data

# Add a main title and descriptive text
# fig.suptitle('Boeing 787 Payload–Range Analysis', fontsize=16, fontweight='bold', y=0.98)
#fig.text(0.5, 0.02, 'Comparison between Airport Planning Manual and Boeing 787 specifications', 
       #  ha='center', fontsize=10, fontstyle='italic')

# Add grid lines to make data more readable
for ax in [ax1]: #, ax2]:
    ax.grid(which='major', linestyle='-', linewidth='0.5', color='gray', alpha=0.5)
    ax.grid(which='minor', linestyle=':', linewidth='0.5', color='lightgray', alpha=0.3)

# Add units information
# ax1.text(0.05, 0.95, 'Units: kg / nautical miles', transform=ax1.transAxes, 
#          fontsize=8, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
legend_elements = [
     plt.Line2D([0], [0], color=colors[0], lw=2.5, marker='o', markersize=6, label='Airport Planning Manual'),
    plt.Line2D([0], [0], color=colors[1], lw=2.5, marker='o', markersize=6, label='ATR72-600 RCAIDE')
]
fig.legend(handles=legend_elements, loc='lower center', ncol=2, frameon=True, 
          fontsize=11, bbox_to_anchor=(0.5, 0.01), framealpha=0.9, 
          edgecolor='gray')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to accommodate title and footer

plt.show()