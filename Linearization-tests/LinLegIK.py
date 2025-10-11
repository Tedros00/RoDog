import numpy as np
import matplotlib.pyplot as plt
import warnings
from matplotlib.widgets import Slider
import argparse

# Create parser
parser = argparse.ArgumentParser(description="Set grid size N and distance threshold h")

# Add arguments with default values
parser.add_argument('--N', type=int, default=100, help='Grid size (default 100)')
parser.add_argument('--h', type=float, default=30, help='Distance threshold (default 30)')

# Parse arguments
args = parser.parse_args()

# Access values
N = args.N
h = args.h

warnings.filterwarnings("ignore", category=RuntimeWarning)

# ---- Parameters ----
Ls = 40.5
Lh = 60
Lk = 100

# ---- Nonlinear IK ----
def nonlinLegIK(EE_x, EE_y, EE_z):
    
    #swing angle
    R0 = np.hypot(EE_y, EE_z)
    alpha = np.acos(z/R0)
    Beta = np.acos(Ls/R0)
    if z > 0:
        theta_S = alpha - Beta
    else:   
        theta_S = np.pi -alpha - Beta

    #hip angle
    yp = np.sqrt(EE_y**2 + EE_z**2 - Ls**2)
    R1 = np.hypot(EE_x, yp)

    phi = np.arccos(EE_x/R1)
    psi = np.arccos((Lh**2 + R1**2 - Lk**2) / (2*Lh*R1))

    if x>0 :
        theta_H = np.pi/2 - phi - psi
    else:
        theta_H = -np.pi/2 + phi - psi 

    #knee angle
    theta_K = np.acos((Lh**2 + Lk**2 - R1**2) / (2*Lh*Lk))
    

    return theta_S, theta_H, theta_K

# ---- Linearized IK ----
def LinLegIK(XYZ0):
    x0, y0, z0 = XYZ0.flatten()

    R0 = np.hypot(y0, z0)
    sgn = -1 #np.sign(x0)
    y0p = np.sqrt(y0**2 + z0**2 - Ls**2)

    # Shoulder
    theta_s_X = 0
    theta_s_Y = (z0)/(R0**2) - (Ls*y0)/(R0**2 * np.sqrt(R0**2 - Ls**2))
    theta_s_Z = -(y0)/(R0**2) - (Ls*z0)/(R0**2 * np.sqrt(R0**2 - Ls**2))
    theta_s_C = (np.arccos(z0/R0) - np.arccos(Ls/R0)) - theta_s_Y*y0 - theta_s_Z*z0

    # Elbow
    u_phi = x0 / (x0**2 + y0p**2)
    phi0 = np.arccos(u_phi)
    u_psi = (Lh**2 + x0**2 + y0p**2 - Lk**2) / (2 * Lh * np.sqrt(x0**2 + y0p**2))
    psi0 = np.arccos(u_psi)

    dphi_dx = -(y0p**2 - x0**2)/((x0**2 + y0p**2)**2 * np.sqrt(1 - u_phi**2))
    dphi_dy = 2*x0*y0 / ((x0**2 + y0p**2)**2 * np.sqrt(1 - u_phi**2))
    dphi_dz = 2*x0*z0 / ((x0**2 + y0p**2)**2 * np.sqrt(1 - u_phi**2))
    dpsi_dx = - x0*(x0**2 + y0p**2 + Lk**2 - Lh**2) / (2*Lh*(x0**2 + y0p**2)**(3/2) * np.sqrt(1 - u_psi**2))
    dpsi_dy = - y0*(x0**2 + y0p**2 + Lk**2 - Lh**2) / (2*Lh*(x0**2 + y0p**2)**(3/2) * np.sqrt(1 - u_psi**2))
    dpsi_dz = - z0*(x0**2 + y0p**2 + Lk**2 - Lh**2) / (2*Lh*(x0**2 + y0p**2)**(3/2) * np.sqrt(1 - u_psi**2))

    theta_h_X = sgn*dphi_dx - dpsi_dx
    theta_h_Y = sgn*dphi_dy - dpsi_dy
    theta_h_Z = sgn*dphi_dz - dpsi_dz
    theta_h_C = -psi0 + sgn*phi0

    # Knee
    u0 = (Lh**2 + Lk**2 - x0**2 - y0p**2) / (2 * Lh * Lk)
    theta_k_X = x0 / (Lh * Lk * np.sqrt(1 - u0**2))
    theta_k_Y = y0 / (Lh * Lk * np.sqrt(1 - u0**2))
    theta_k_Z = z0 / (Lh * Lk * np.sqrt(1 - u0**2))
    theta_k_C = np.arccos(u0)

    A = np.array([[theta_s_X, theta_s_Y, theta_s_Z],
                  [theta_h_X, theta_h_Y, theta_h_Z],
                  [theta_k_X, theta_k_Y, theta_k_Z]])
    
    B = np.array([[theta_s_C], [theta_h_C], [theta_k_C]])
    return A, B

# ---- Forward Kinematics ----
def legFK(theta_s, theta_h, theta_k):
    
    #base to shoulder

    theta_k = np.pi - theta_k

    Rsh = np.array( [ [1 , 0 , 0 , 0] ,
            [0, np.cos(theta_s), -np.sin(theta_s), 0] ,
            [0, np.sin(theta_s), np.cos(theta_s), Ls] ,
            [0 , 0 , 0 , 1] ] )
    
    Rhk = np.array( [ [np.cos(theta_h), 0, np.sin(theta_h), 0] , 
                      [np.sin(theta_h), np.cos(theta_h), 0, Lh] ,
                      [0 , 0 , 1, 0] ,
                      [0 , 0, 0, 1] ] )
    
    Rkee = np.array( [ [np.cos(theta_k), 0, np.sin(theta_k), 0] , 
                      [np.sin(theta_k), np.cos(theta_k), 0, Lk] ,
                      [0 , 0 , 1, 0] ,
                      [0 , 0, 0, 1] ] )
    

    R0 = Rsh @ Rhk @ Rkee
    x0 = R0[0, 3]   
    y0 = R0[1, 3]
    z0 = R0[2, 3]   

    return x0, y0, z0

# ---- Main Simulation ----
XYZ0 = np.array([[0], [120], [40.5]])
A, B = LinLegIK(XYZ0)


Xfield = np.linspace(-40, 40, N)
Yfield = np.linspace(80, 160, N)
Zfield = np.linspace(10, 90, N)

linErr = np.zeros((len(Xfield), len(Yfield), len(Zfield)))
nonlinErr = np.zeros((len(Xfield), len(Yfield), len(Zfield)))

linearization_updates = 0

for k, z in enumerate(Zfield):
    for i, x in enumerate(Xfield):
        for j, y in enumerate(Yfield):
            try:
                theta_s_nl, theta_h_nl, theta_k_nl = nonlinLegIK(x, y, z)
            except ValueError:
                continue

            XYZ = np.array([[x], [y], [z]])

            dist = np.linalg.norm(XYZ - XYZ0)
            if dist > h:
                #print(f"Updating linearization at point ({x:.2f}, {y:.2f}, {z:.2f}), distance {dist:.2f}")
                linearization_updates += 1
                XYZ0 = XYZ
                A, B = LinLegIK(XYZ0)

            theta = A @ XYZ + B
            theta_s_l, theta_h_l, theta_k_l = theta.flatten()

            # Forward kinematics
            x_fk_nl, y_fk_nl, z_fk_nl = legFK(theta_s_nl, theta_h_nl, theta_k_nl)
            x_fk_l, y_fk_l, z_fk_l = legFK(theta_s_l, theta_h_l, theta_k_l)

            # Compute errors (Euclidean distance)
            nonlinErr[i, j, k] = np.linalg.norm([x - x_fk_nl, y - y_fk_nl, z - z_fk_nl])
            linErr[i, j, k] = np.linalg.norm([x - x_fk_l, y - y_fk_l, z - z_fk_l])


print(f"Number of linearization updates: {linearization_updates}")
# Initial mid slice
midZ_val = 40.5
midZ_idx = np.argmin(np.abs(Zfield - midZ_val))

# Determine common color scale limits
vmin = min(np.nanmin(linErr), np.nanmin(nonlinErr))
vmax = max(np.nanmax(linErr), np.nanmax(nonlinErr))

fig, axes = plt.subplots(1, 2, figsize=(12, 5))
plt.subplots_adjust(bottom=0.25)  # space for slider

# Initial plots
im1 = axes[0].imshow(linErr[:, :, midZ_idx].T, origin='lower',
                     extent=[Xfield[0], Xfield[-1], Yfield[0], Yfield[-1]],
                     cmap='magma', vmin=vmin, vmax=vmax)
axes[0].set_title('Linear IK Error (slice)')
axes[0].set_xlabel('X')
axes[0].set_ylabel('Y')

im2 = axes[1].imshow(nonlinErr[:, :, midZ_idx].T, origin='lower',
                     extent=[Xfield[0], Xfield[-1], Yfield[0], Yfield[-1]],
                     cmap='magma', vmin=vmin, vmax=vmax)
axes[1].set_title('Nonlinear IK Error (slice)')
axes[1].set_xlabel('X')
axes[1].set_ylabel('Y')

fig.text(0.15, 0.17, f'Linearization distance threshold: {h}\nLinearization updates: {linearization_updates} \n Grid size: {N}x{N}x{N}', 
         ha='left', va='top', fontsize=10)

# Single colorbar on right
cbar = fig.colorbar(im2, ax=axes[1], orientation='vertical')
cbar.set_label('Error magnitude')

# Display current Z
z_text = fig.text(0.5, 0.1, f'Current Z = {Zfield[midZ_idx]:.2f}', ha='center')

# Slider axes
ax_slider = plt.axes([0.25, 0.05, 0.5, 0.03])
slider = Slider(ax_slider, 'Z slice', Zfield[0], Zfield[-1], valinit=Zfield[midZ_idx])

# Update function
def update(val):
    z_val = slider.val
    midZ_idx = np.argmin(np.abs(Zfield - z_val))
    im1.set_data(linErr[:, :, midZ_idx].T)
    im2.set_data(nonlinErr[:, :, midZ_idx].T)
    z_text.set_text(f'Current Z = {Zfield[midZ_idx]:.2f}')
    fig.canvas.draw_idle()

slider.on_changed(update)

plt.show()
