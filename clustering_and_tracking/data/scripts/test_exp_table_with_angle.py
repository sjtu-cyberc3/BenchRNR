# Import necessary libraries
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from IPython.display import display
import matplotlib.pyplot as plt

# Function to read data from file
def read_data(file_path):
    """Read data from a txt file"""
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            timestamp = float(parts[0])
            cosx = float(parts[-2])
            sinx = float(parts[-1])
            x = float(parts[1])
            y = float(parts[2])
            data.append((timestamp, cosx, sinx, x , y))
    return np.array(data)

# Function to align data based on timestamp
def align_data(data1, data2, threshold=0.1):
    """Align data based on timestamp"""
    aligned_data1, aligned_data2 = [], []
    for i in range(len(data2)):
        closest_index = np.argmin(np.abs(data1[:, 0] - data2[i, 0]))
        if abs(data1[closest_index, 0] - data2[i, 0]) < threshold:
            aligned_data1.append(data1[closest_index])
            aligned_data2.append(data2[i])
    
    return np.array(aligned_data1), np.array(aligned_data2)

# Function to calculate angle error
def calculate_angle_error(cos1, sin1, cos2, sin2, type="detect"):
    """Calculate the angular error between two vectors (in degrees)"""
    angle1 = np.arctan2(-cos1, sin1)
    angle2 = np.arctan2(sin2, cos2)
    angle_diff = np.abs(angle1 - angle2)
    angle_diff = np.minimum(angle_diff, 2 * np.pi - angle_diff) * (180 / np.pi)
    
    angle_diff = np.where(abs(180 - angle_diff) < angle_diff, abs(180 - angle_diff), angle_diff)
    
    if type != "DL":
        angle_diff = np.where(abs(90 - angle_diff) < angle_diff, abs(90 - angle_diff), angle_diff)
    
    return angle_diff

# Function to calculate statistics
def calculate_statistics(n):
    """Compute mean, median, variance, and max of angle n"""
    n_filtered = n[~np.isnan(n)]
    
    lower_bound = np.nanmean(n) - 1.96 * np.nanstd(n)
    upper_bound = np.nanmean(n) + 1.96 * np.nanstd(n)
        
    n = n[n <= upper_bound]

    mean_error = np.mean(n)
    median_error = np.median(n)
    variance_error = np.var(n)
    max_error = np.max(n)
    
    return mean_error, median_error, variance_error, max_error

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from IPython.display import display

def plot_error_comparison(df):
    """Plot bar charts comparing mean errors in localization and heading estimation."""
    
    # Define the color palette for each str_num (m, n, m_64)
    ccolor = ['#0078ff', "#64c8c8", "#b8f1ed"]  # Colors for m, n, m_64

    # Extract mean error values from the DataFrame, grouped by method and str_num
    methods = ['Seg+obb', 'Seg+convex', 'Register_loc', 'PV-RCNN']
    str_nums = ['m', 'n', 'm_64']
    
    # Create lists to store errors for each method and str_num combination
    rep_loc = []
    rep_ang = []
    
    # Loop through methods and str_nums to get the corresponding errors
    for method in methods:
        method_loc = []
        method_ang = []
        for str_num in str_nums:
            mean_distance_error = df[(df['method'] == method) & (df['str_num'] == str_num)]['mean_distance_error'].values
            mean_angle_error = df[(df['method'] == method) & (df['str_num'] == str_num)]['mean_angle_error'].values
            
            # Append the errors for each method and str_num
            method_loc.append(mean_distance_error[0] if len(mean_distance_error) > 0 else 0)
            method_ang.append(mean_angle_error[0] if len(mean_angle_error) > 0 else 0)
        
        rep_loc.append(method_loc)
        rep_ang.append(method_ang)

    # Convert lists to numpy arrays for plotting
    rep_loc = np.array(rep_loc) * 100
    rep_ang = np.array(rep_ang) * 100

    # Set positions for the bars
    index = np.arange(len(rep_loc))  # Now index is of length 4
    bar_width = 0.18  # Adjusted the bar width to fit three bars per group

    # Create a figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))  # 1 row, 2 columns

    # Plot the first subplot (Localization Errors)
    ax1.bar(index - bar_width, rep_loc[:, 0], bar_width, label='Repetitive Scanning (128)', color=ccolor[0], alpha=1)
    ax1.bar(index, rep_loc[:, 1], bar_width, label='Non-repetitive Scanning', color=ccolor[1], alpha=1)
    ax1.bar(index + bar_width, rep_loc[:, 2], bar_width, label='Repetitive Scanning (64)', color=ccolor[2], alpha=1)
    
    # Add values on top of the bars in the first subplot
    for i in range(len(methods)):
        ax1.text(index[i] - bar_width, rep_loc[i, 0], f'{rep_loc[i, 0]:.2f}', ha='center', va='bottom', fontsize=8)
        ax1.text(index[i], rep_loc[i, 1], f'{rep_loc[i, 1]:.2f}', ha='center', va='bottom', fontsize=8)
        ax1.text(index[i] + bar_width, rep_loc[i, 2], f'{rep_loc[i, 2]:.2f}', ha='center', va='bottom', fontsize=8)

    ax1.set_xlabel('Methods')
    ax1.set_ylabel('Localization Error (m)')
    ax1.set_xticks(index)
    ax1.set_xticklabels(methods)
    ax1.yaxis.grid(True)

    # Plot the second subplot (Heading Estimation Errors)
    ax2.bar(index - bar_width, rep_ang[:, 0], bar_width, label='Repetitive Scanning (128)', color=ccolor[0], alpha=1)
    ax2.bar(index, rep_ang[:, 1], bar_width, label='Non-repetitive Scanning', color=ccolor[1], alpha=1)
    ax2.bar(index + bar_width, rep_ang[:, 2], bar_width, label='Repetitive Scanning (64)', color=ccolor[2], alpha=1)
    
    # Add values on top of the bars in the second subplot
    for i in range(len(methods)):
        ax2.text(index[i] - bar_width, rep_ang[i, 0], f'{rep_ang[i, 0]:.2f}', ha='center', va='bottom', fontsize=8)
        ax2.text(index[i], rep_ang[i, 1], f'{rep_ang[i, 1]:.2f}', ha='center', va='bottom', fontsize=8)
        ax2.text(index[i] + bar_width, rep_ang[i, 2], f'{rep_ang[i, 2]:.2f}', ha='center', va='bottom', fontsize=8)

    ax2.set_xlabel('Methods')
    ax2.set_ylabel('Heading Estimation Error (deg)')
    ax2.set_xticks(index)
    ax2.set_xticklabels(methods)
    ax2.yaxis.grid(True)

    # Add a single legend for all bars in the right corner (lower right)
    ax1.legend(loc="lower right", title='Scan Type')
    ax2.legend(loc='lower right', title='Scan Type')

    # Adjust the layout to avoid overlap
    plt.tight_layout()

    # Save the plot
    plt.savefig('error_comparison_plot.png', dpi=300, bbox_inches='tight')

    # Display the plot
    plt.show()

# Main function to process the data and return results
def main_api(str_num, method="Seg+obb"):
    # File paths
    file1 = 'combined_txt/gt.txt'
    
    
    if method == "Seg+obb":
        file2 = 'combined_txt/detect_' + str_num + '.txt'
    elif method == "Seg+convex":
        file2 = 'combined_txt/detect_convex_' + str_num + '.txt'    
    elif method == "PV-RCNN":
        file2 = 'combined_txt/DL_' + str_num + '.txt'
    elif method == "Register_loc":
        file2 = 'combined_txt/regist_' + str_num + '.txt'
    
    print(file2)

    # Read data
    data1 = read_data(file1)
    data2 = read_data(file2)

    # Align data
    aligned_data1, aligned_data2 = align_data(data1, data2)
    
    # Calculate angle n between aligned data1 and data2
    try:
        timestamps = aligned_data1[:, 0]
    except IndexError:
        print(f"Error: Aligned data for {str_num} with method {method} is empty or misaligned.")
        return {
            "str_num": str_num,
            "method": method,
            "mean_distance_error": None,
            "max_distance_error": None,
            "std_distance_error": None,
            "mean_angle_error": None,
            "variance_angle_error": None,
            "max_angle_error": None,
            "total_frames": 0
        }
    n = calculate_angle_error(aligned_data1[:, 1], aligned_data1[:, 2],
                                   aligned_data2[:, 1], aligned_data2[:, 2], method)
    
    dist_error = np.sqrt((aligned_data1[:, 3] - aligned_data2[:, 3])**2 + (aligned_data1[:, 4] - aligned_data2[:, 4])**2)
    
    dist_error = dist_error[dist_error < 2]  # Filter out infinite values
    
    dist_error = dist_error[~np.isnan(dist_error)]  # Filter out NaN values
    
    # Calculate mean and std deviation
    mean = np.mean(dist_error)
    std = np.std(dist_error)

    # Compute upper and lower bounds
    upper_bound = mean + 1.96 * std

    # Filter data
    filtered_error = dist_error[dist_error <= upper_bound]
    
    # Compute angle statistics
    mean_error, median_error, variance_error, max_error = calculate_statistics(n)
    
    return {
        "str_num": str_num,
        "method": method,
        "mean_distance_error": np.mean(filtered_error),
        "max_distance_error": np.max(filtered_error),
        "std_distance_error": np.std(filtered_error),
        "mean_angle_error": mean_error,
        "variance_angle_error": variance_error,
        "max_angle_error": max_error,
        "total_frames": len(timestamps)
    }

# Collect results for all combinations of str_num and method
results = []

str_nums = ["m", "n", "m_64"]
methods = ["Seg+obb", "Seg+convex", "Register_loc", "PV-RCNN"]

for str_num in str_nums:
    for method in methods:
        result = main_api(str_num, method)
        results.append(result)

# Convert results to pandas DataFrame
df = pd.DataFrame(results)

# Round all numerical values to 3 decimal places
df = df.round(4)

# Call the plotting function
plot_error_comparison(df)

# Display the dataframe in Jupyter Notebook
display(df)
