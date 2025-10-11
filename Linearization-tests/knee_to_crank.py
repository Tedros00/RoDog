import numpy as np
import matplotlib.pyplot as plt

# Given data converted to radians
crank_angle = np.array([0.0000, 0.1745, 0.3491, 0.5236, 0.6981,
                        0.8727, 1.0472, 1.2217, 1.3963, 1.5708])
knee_angle  = np.array([0.6319, 0.6413, 0.6988, 0.7978, 0.9305,
                        1.0868, 1.2587, 1.4407, 1.6276, 1.8190])

# Fitted parameters (in radians)
A = 0.8758
B = 1.2419
phi = -1.5852
D = 1.5026

# Define inverse function: crank_angle = F(knee_angle)
def F(k):
    x = np.clip((k - D) / A, -1, 1)
    return (np.arcsin(x) - phi) / B

# Generate smooth knee-angle range
k_smooth = np.linspace(min(knee_angle), max(knee_angle), 300)
c_smooth = F(k_smooth)

# Plot crank_angle = F(knee_angle)
plt.figure(figsize=(7, 5))
plt.scatter(knee_angle, crank_angle, color='red', label='Given Data', zorder=3)
plt.plot(k_smooth, c_smooth, color='blue', linewidth=2, label='Fitted Function')
plt.xlabel('Knee Angle (radians)', fontsize=12)
plt.ylabel('Crank Angle (radians)', fontsize=12)
plt.title('Crank Angle as a Function of Knee Angle (radians)')
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
