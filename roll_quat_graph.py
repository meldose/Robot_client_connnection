import math

def quaternion_to_euler(w, x, y, z):
    # Roll (X-axis rotation)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))

    # Pitch (Y-axis rotation)
    pitch = math.asin(2.0 * (w * y - z * x))

    # Yaw (Z-axis rotation)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

    return roll, pitch, yaw

# Example usage:
w, x, y, z = 0.7071, 0.7071, 0.0, 0.0  # Example quaternion
roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
print("Roll:", roll, "radians")
print("Pitch:", pitch, "radians")
print("Yaw:", yaw, "radians")
