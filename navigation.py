"""
Navigation simulation of rocket trajectory 
@author: leapedcamera
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt

import gnsstoolbox.orbits as orb
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
from demo_algorithms import ins_loose, ins_tight, free_integration


# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('./data/')
imu_fs = 100.0          # IMU sample frequency
gps_fs = 1.0            # GPS sample frequency

'''
test Sim
'''
#### IMU model, typical for IMU381
imu_err = {'gyro_b': np.array([0.0, 0.0, 0.0]),
            'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,
            'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,
            'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
            'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
            'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,
            'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,
            'accel_b_corr': np.array([200.0, 200.0, 200.0]),
            'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0
            }
odo_err = {'scale': 0.999,
            'stdv': 0.1}

gps_err = { 'mode': 'tight',
            'stdp': np.array([5.0, 5.0, 7.0]),
            'stdv': np.array([0.05, 0.05, 0.05])}
# do not generate GPS and magnetometer data
imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=True, gps_opt=gps_err, odo=True, odo_opt=odo_err)


ini_pos_vel_att = np.genfromtxt(motion_def_path+"/rocket.csv",\
                                delimiter=',', skip_header=1, max_rows=1)
ini_pos_vel_att[0] = ini_pos_vel_att[0] * D2R
ini_pos_vel_att[1] = ini_pos_vel_att[1] * D2R
ini_pos_vel_att[6:9] = ini_pos_vel_att[6:9] * D2R

# add initial states error if needed
ini_vel_err = np.array([0.0, 0.0, 0.0]) # initial velocity error in the body frame, m/s
ini_att_err = np.array([0.0, 0.0, 0.0]) # initial Euler angles error, deg
ini_pos_vel_att[3:6] += ini_vel_err
ini_pos_vel_att[6:9] += ini_att_err * D2R

#### Load GPS simulator
gps_orbits = orb.orbit()
rinex=[ motion_def_path+"/AMC400USA_R_20230290000_15M_GN.rnx",\
                           motion_def_path+"/BREW00USA_R_20230290000_15M_GN.rnx",\
                           motion_def_path+"/USUD00JPN_R_20230290000_15M_GN.rnx",\
                           motion_def_path+"/WROC00POL_R_20230290000_15M_GN.rnx",\
                           motion_def_path+"/YEL200CAN_R_20230290000_15M_GN.rnx",\
                           motion_def_path+"/YKRO00CIV_R_20230290000_15M_GN.rnx",\
                           motion_def_path+"/ZAMB00ZMB_R_20230290000_15M_GN.rnx",\
                           motion_def_path+"/VACS00MUS_R_20230290000_15M_GN.rnx"     ]
for iRin in rinex:
    gps_orbits.loadRinexN(iRin)

# create the algorith object
algo1 = free_integration.FreeIntegration(ini_pos_vel_att)
algo2 = ins_tight.InsTight(gps_orbits, True)

#### Setup simulation
sim = ins_sim.Sim([imu_fs, gps_fs, 0.0],
                    motion_def_path+"/rocket.csv",
                    ref_frame=1,
                    imu=imu,
                    mode=None,
                    env=None,
                    algorithm=[algo1, algo2],
                    orbit=gps_orbits )

# run the simulation for 3 times
sim.run(3)

# generate simulation results, summary
t = np.array(sim.dmgr.time.data)
pos = np.array(sim.dmgr.ref_pos.data)
quat = np.array(sim.dmgr.ref_att_quat.data)
att = np.array(sim.dmgr.ref_att_euler.data)

# Convert to ENU
pos[:,2] = -1 * pos[:,2]
temp = np.array(pos[:,0])
pos[:,0] = pos[:,1]
pos[:,1] = temp

# Only look at delta 
pos[:,0] = pos[:,0] - pos[0,0]
pos[:,1] = pos[:,1] - pos[0,1]
pos[:,2] = pos[:,2] - pos[0,2] + ini_pos_vel_att[2]

# Plot the truth trajectory from the sim
max_z = max(pos[:, 2])
min_z = min(pos[:, 2])
max_x = max(pos[:, 0])
min_x = min(pos[:, 0])
max_y = max(pos[:, 1])
min_y = min(pos[:, 1])
min_xy = min(min_x, min_y)
max_xy = max(max_x, max_y)

# avoids errors when x_lim and y_lim are the same
if abs(min_z - max_z) < 1e-5:
    max_z += 1
if abs(min_xy - max_xy) < 1e-5:
    max_xy += 1

plt.plot(t, pos[:,2])

_ = plt.figure(figsize=(9, 9))
ax1 = plt.subplot(111, projection="3d")
ax1.plot(
    pos[:, 0],
    pos[:, 1], 
    zs=min_z, 
    zdir="z", 
    linestyle="--"
)
ax1.plot(
    pos[:, 0],
    pos[:, 2],
    zs=min_y,
    zdir="y",
    linestyle="--",
)
ax1.plot(
    pos[:, 1],
    pos[:, 2],
    zs=min_x,
    zdir="x",
    linestyle="--",
)
ax1.plot(
    pos[:, 0],
    pos[:, 1],
    pos[:, 2],
    linewidth="2",
)
ax1.set_xlabel("X - East (m)")
ax1.set_ylabel("Y - North (m)")
ax1.set_zlabel("Z - Altitude Above Ground Level (m)")
ax1.set_title("Flight Trajectory")
ax1.set_xlim(min_xy, max_xy)
ax1.set_ylim(min_xy, max_xy)
ax1.set_zlim(min_z, max_z)
ax1.view_init(15, 45)
ax1.set_box_aspect(None, zoom=0.95)  # 95% for label adjustment
plt.show()

sim.results(err_stats_start=-1, gen_kml=True)

# plot postion error
sim.plot(['pos'], opt={'pos':'error', 'monte':'true'})
sim.plot(['vel'], opt={'vel':'error'})

plt.show()