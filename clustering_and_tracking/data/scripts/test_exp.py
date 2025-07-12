import numpy as np
import matplotlib.pyplot as plt

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

def align_data(data1, data2, threshold=0.1):
    """Align data based on timestamp"""
    aligned_data1, aligned_data2 = [], []
    # Iterate through data2 and find the closest timestamp in data1
    for i in range(len(data2)):
        # Find the closest timestamp in data1
        closest_index = np.argmin(np.abs(data1[:, 0] - data2[i, 0]))
        if abs(data1[closest_index, 0] - data2[i, 0]) < threshold:
            aligned_data1.append(data1[closest_index])
            aligned_data2.append(data2[i])
    
    return np.array(aligned_data1), np.array(aligned_data2)

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

def plot_angle_errors(timestamps, errors):
    """Plot the angle error over time"""
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, errors, marker='o', linestyle='-', color='b')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Angle Error (degrees)')
    plt.title('Angle Error Between Two Data Sets')
    plt.grid(True)
    plt.show()

def calculate_statistics(errors):
    """Compute mean, median, variance, and max of angle errors"""
    # Compute upper and lower bounds
    lower_bound = np.mean(errors) - 1.96 * np.std(errors)
    upper_bound = np.mean(errors) + 1.96 * np.std(errors)    
    # Filter data
    errors = errors[errors <= upper_bound]
        
    mean_error = np.mean(errors)
    median_error = np.median(errors)
    variance_error = np.var(errors)
    max_error = np.max(errors)
    
    return mean_error, median_error, variance_error, max_error

def main_api(str_num, method="Seg+obb"):
    # File paths
    file1 = 'combined_txt/gt.txt'
    
    if method == "Seg+obb":
        file2 = 'combined_txt/detect_' + str_num + '.txt'
    elif method == "Seg+convex":
        file2 = 'combined_txt/detect_convex_' + str_num + '.txt'    
    elif method == "DL":
        file2 = 'combined_txt/DL_' + str_num + '.txt'
    elif method == "Register_loc":
        file2 = 'combined_txt/regist_' + str_num + '.txt'

    # Read data
    data1 = read_data(file1)
    data2 = read_data(file2)

    # Align data
    aligned_data1, aligned_data2 = align_data(data1, data2)
    
    # Calculate angle errors
    timestamps = aligned_data1[:, 0]
    errors = calculate_angle_error(aligned_data1[:, 1], aligned_data1[:, 2],
                                   aligned_data2[:, 1], aligned_data2[:, 2], method)
    
    dist_error = np.sqrt((aligned_data1[:, 3] - aligned_data2[:, 3])**2 + (aligned_data1[:, 4] - aligned_data2[:, 4])**2)
    print("====================================")
    
    print(method)
    
    # Calculate mean and std deviation
    mean = np.mean(dist_error)
    std = np.std(dist_error)

    # Compute upper and lower bounds
    upper_bound = mean + 1.96 * std

    # Filter data
    filtered_error = dist_error[dist_error <= upper_bound]
    
    print("Distance error:")
    print("Mean: ", np.mean(filtered_error))
    print("Max: ", np.max(filtered_error))
    print("Std: ", np.std(filtered_error))
    
    if method == "DL":  # Filter data with large x, y errors and save
        print("DL data")
        dist = np.sqrt((aligned_data1[:, 3] - aligned_data2[:, 3])**2 + (aligned_data1[:, 4] - aligned_data2[:, 4])**2)
        mask = dist < 2
        
        filtered_data1 = aligned_data2[mask]
        
        # Save filtered data
        save_path = 'combined_txt/filtered_DL_data_' + str_num + '.txt'
        
        with open(save_path, 'w') as f:
            for row in filtered_data1:
                f.write(f"{row[0]} {row[3]} {row[4]} 0 0 0 {row[1]} {row[2]}\n")
        
    # plot_angle_errors(timestamps, errors)

    # Compute statistics
    mean_error, median_error, variance_error, max_error = calculate_statistics(errors)
    print("-------------------")
    print("Heading estimation error:")
    print(f"Mean angle error: {mean_error:.4f} degrees")
    print(f"Angle error variance: {variance_error:.4f}")
    print(f"Max angle error: {max_error:.4f} degrees")
    print("Total frames: ", len(timestamps))

# Main entry
if __name__ == "__main__":    
    main_api("m_64", "Seg+obb")
    main_api("m_64", "Seg+convex")
    main_api("m_64", "Register_loc")
