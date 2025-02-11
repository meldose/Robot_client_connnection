def calculate_target_position(Tx, delta_MSD, delta_TSC, delta_L, delta_CTD):
    """
    Calculate the target X position on the conveyor belt.

    Parameters:
    Tx (float): Initial X position of the target (in mm or m).
    delta_MSD (float): Marker Space Displacement (in mm or m).
    delta_TSC (float): Trigger to Scan Completion distance (in mm or m).
    delta_L (float): Localization distance (in mm or m).
    delta_CTD (float): Conveyor Tracking Distance (in mm or m).

    Returns:
    float: Calculated Target X position.
    """
    TD = delta_TSC + delta_L + delta_CTD
    target_X = Tx - delta_MSD + TD
    return target_X

# Example input values
Tx = 150.0          # Initial position in mm
Delta_MSD = 10.0    # Marker Space Displacement in mm
Delta_TSC = 20.0    # Trigger to Scan Completion in mm
Delta_L = 30.0      # Localization distance in mm
Delta_CTD = 40.0    # Conveyor Tracking Distance in mm

# Calculate Target X position
target_position = calculate_target_position(Tx, Delta_MSD, Delta_TSC, Delta_L, Delta_CTD)

print(f"Calculated Target X Position: {target_position} mm")
