import numpy as np
from scipy.optimize import minimize

# Initial rough calibration values (replace with actual values)
initial_params = {
    'gear_reduction_1': 100.0,
    'gear_reduction_2': 100.0,
    'arm_length_1': 150.0,
    'arm_length_2': 150.0,
    'arm_1_home': 0.0,
    'arm_2_home': 0.0,
}

# Function to calculate expected position using inverse kinematics
def calculate_expected_position(params, angles):
    # Extract parameters
    gear_reduction_1 = params['gear_reduction_1']
    gear_reduction_2 = params['gear_reduction_2']
    arm_length_1 = params['arm_length_1']
    arm_length_2 = params['arm_length_2']
    arm_1_home = params['arm_1_home']
    arm_2_home = params['arm_2_home']
    
    # Convert angles to joint positions
    theta1 = angles[0] / gear_reduction_1 + arm_1_home
    theta2 = angles[1] / gear_reduction_2 + arm_2_home
    
    # Forward kinematics to find (x, y)
    x = arm_length_1 * np.cos(theta1) + arm_length_2 * np.cos(theta1 + theta2)
    y = arm_length_1 * np.sin(theta1) + arm_length_2 * np.sin(theta1 + theta2)
    
    return np.array([x, y])

# Error function
def error_function(params_array, test_points, actual_positions):
    # Convert params array to dictionary
    params = {
        'gear_reduction_1': params_array[0],
        'gear_reduction_2': params_array[1],
        'arm_length_1': params_array[2],
        'arm_length_2': params_array[3],
        'arm_1_home': params_array[4],
        'arm_2_home': params_array[5],
    }
    
    total_error = 0.0
    for i, angles in enumerate(test_points):
        expected_position = calculate_expected_position(params, angles)
        actual_position = actual_positions[i]
        total_error += np.sum((expected_position - actual_position) ** 2)
    
    return total_error

# Example test points and actual positions (replace with actual data)
test_points = np.array([
    [10, 20],
    [30, 40],
    # More test points...
])

actual_positions = np.array([
    [0.5, 1.0],
    [1.5, 2.0],
    # More actual positions...
])

# Initial parameter array for optimization
initial_params_array = [
    initial_params['gear_reduction_1'],
    initial_params['gear_reduction_2'],
    initial_params['arm_length_1'],
    initial_params['arm_length_2'],
    initial_params['arm_1_home'],
    initial_params['arm_2_home'],
]

# Optimization
result = minimize(error_function, initial_params_array, args=(test_points, actual_positions), method='BFGS')

# Optimized parameters
optimized_params = result.x

# Print optimized parameters
print("Optimized Parameters:")
print("Gear Reduction 1:", optimized_params[0])
print("Gear Reduction 2:", optimized_params[1])
print("Arm Length 1:", optimized_params[2])
print("Arm Length 2:", optimized_params[3])
print("Arm 1 Home:", optimized_params[4])
print("Arm 2 Home:", optimized_params[5])
