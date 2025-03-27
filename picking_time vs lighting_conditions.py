import matplotlib.pyplot as plt
import numpy as np

# Define categories (items) and conditions (light/dark)
items = ['Trapezoid', 'Pipe']
conditions = ['Light', 'Dark']

# Simulated success rates (in percentage)
success_rates = {
    'Trapezoid': {'Bright': 90, 'Dark': 80},
    'Pipe': {'Bright': 80, 'Dark': 60}
}

# Extract values for the bar plot
x = np.arange(len(items))  # the label locations
width = 0.35  # width of the bars

fig, ax = plt.subplots()

# Create bars for light and dark conditions
bars_light = ax.bar(x - width/2, [success_rates[item]['Bright'] for item in items], width, label='Bright')
bars_dark = ax.bar(x + width/2, [success_rates[item]['Dark'] for item in items], width, label='Dark')

# Labels and formatting
ax.set_xlabel('Item Type')
ax.set_ylabel('Picking Success Rate (%)')
ax.set_title('Picking Success Rate Under Bright and Dark Conditions')
ax.set_xticks(x)
ax.set_xticklabels(items)
ax.legend()

# Display value labels on bars
def add_labels(bars):
    for bar in bars:
        height = bar.get_height()
        ax.annotate(f'{height}%',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),  # Offset label above the bar
                    textcoords="offset points",
                    ha='center', va='bottom')

add_labels(bars_light)
add_labels(bars_dark)

plt.show()
