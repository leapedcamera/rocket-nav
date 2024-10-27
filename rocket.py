from rocketpy import Environment, Rocket, SolidMotor, Flight
import datetime, numpy as np
import matplotlib.pyplot as plt
from  gnss_ins_sim.attitude import attitude
import math

env = Environment(
    latitude=32.990254,
    longitude=-106.974998,
    elevation=1400,
) 

tomorrow = datetime.date.today() + datetime.timedelta(days=1)

env.set_date(
  (tomorrow.year, tomorrow.month, tomorrow.day, 12), timezone="America/Denver"
) # Tomorrow's date in year, month, day, hour UTC format

env.set_atmospheric_model(type='Forecast', file='GFS')

Pro75M1670 = SolidMotor(
    thrust_source="data/motors/Cesaroni_M1670.eng",
    dry_mass=1.815,
    dry_inertia=(0.125, 0.125, 0.002),
    center_of_dry_mass_position=0.317,
    grains_center_of_mass_position=0.397,
    burn_time=3.9,
    grain_number=5,
    grain_separation=0.005,
    grain_density=1815,
    grain_outer_radius=0.033,
    grain_initial_inner_radius=0.015,
    grain_initial_height=0.12,
    nozzle_radius=0.033,
    throat_radius=0.011,
    interpolation_method="linear",
    nozzle_position=0,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

calisto = Rocket(
    radius=0.0635,
    mass=14.426,  # without motor
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="data/calisto/powerOffDragCurve.csv",
    power_on_drag="data/calisto/powerOnDragCurve.csv",
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose",
)

buttons = calisto.set_rail_buttons(
    upper_button_position=0.0818,
    lower_button_position=-0.6182,
    angular_position=45,
)

calisto.add_motor(Pro75M1670, position=-1.255)

nose = calisto.add_nose(
    length=0.55829, kind="vonKarman", position=1.278
)

fins = calisto.add_trapezoidal_fins(
    n=4,
    root_chord=0.120,
    tip_chord=0.040,
    span=0.100,
    sweep_length=None,
    cant_angle=0,
    position=-1.04956,
)

tail = calisto.add_tail(
    top_radius=0.0635, bottom_radius=0.0435, length=0.060, position=-1.194656
)

main = calisto.add_parachute(
    name="main",
    cd_s=10.0,
    trigger=800,  # ejection altitude in meters
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)

drogue = calisto.add_parachute(
    name="drogue",
    cd_s=1.0,
    trigger="apogee",  # ejection at apogee
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)

test_flight = Flight(
  rocket=calisto, environment=env, rail_length=5.2, inclination=81, heading=0
)



# Trajectory conversion for GNSS-INS-Sim project

# Initialize results
t = test_flight.e0.x_array
alt = test_flight.z.y_array

plt.plot(t,alt)
v_b = np.zeros( (len(test_flight.e0), 3), dtype=np.float64 )
a_b = np.zeros( (len(test_flight.e0), 3), dtype=np.float64 )
ypr = np.zeros( (len(test_flight.e0), 3), dtype=np.float64 )

# Attitude
e0 = test_flight.e0.y_array
e1 = test_flight.e1.y_array
e2 = test_flight.e2.y_array
e3 = test_flight.e3.y_array
quat = np.column_stack((e0,e1,e2,e3))

# Velocity
vx = test_flight.vx.y_array
vy = test_flight.vy.y_array
vz = test_flight.vz.y_array
v_n = np.column_stack((vx,vy,vz))

# Acceleration
ax = test_flight.ax.y_array
ay = test_flight.ay.y_array
az = test_flight.az.y_array
a_n = np.column_stack((ax,ay,az))

rocketToBody = np.zeros( (3,3), dtype=np.float64)
rocketToBody[0, 2] = 1
rocketToBody[1, 0] = -1
rocketToBody[2, 1] = -1

nedToEnu = np.zeros( (3,3), dtype=np.float64)
nedToEnu[0, 1] = 1
nedToEnu[1, 0] = 1
nedToEnu[2, 2] = -1

for idx, x in enumerate(test_flight.e0):
    enuToRocket = attitude.quat2dcm(quat[idx,:])
    nedToBody = np.matmul( np.matmul( rocketToBody, enuToRocket ), nedToEnu )
    ypr[idx,:] = attitude.dcm2euler( nedToBody ) * 180 / math.pi
    v_b[idx,:] = np.transpose( np.matmul( np.matmul(rocketToBody, enuToRocket), np.transpose(v_n[idx,:])))
    a_b[idx,:] = np.transpose( np.matmul( np.matmul(rocketToBody, enuToRocket), np.transpose(a_n[idx,:])))


plt.figure(figsize=(9, 6))
ax1 = plt.subplot(311)
ax1.plot(
    t,
    ypr[:,0],
    label="Yaw",
)
ax1.plot(
    t,
    ypr[:,1],
    label="Pitch",
)
ax1.plot(
    t,
    ypr[:,2],
    label="Roll",
)
ax1.legend()
ax1.grid(True)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Angle (°)")
ax1.set_title("Flight Attitude")

ax2 = plt.subplot(312)
ax2.plot(
    t,
    v_b[:,0],
)
ax2.plot(
    t,
    v_b[:,1],
)
ax2.plot(
    t,
    v_b[:,2],
)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Speed (m/s)")
ax2.set_title("Body Velocity")
ax2.grid(True)

ax3 = plt.subplot(313)
ax3.plot(
    t,
    a_b[:,0],
)
ax3.plot(
    t,
    a_b[:,1],
)
ax3.plot(
    t,
    a_b[:,2],
)
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Speed (m/s)")
ax3.set_title("Body Acceleration")
ax3.grid(True)

plt.subplots_adjust(hspace=0.5)
plt.show()
test_flight.info()
test_flight.all_info()

import csv
with open('./data/rocket.csv', 'w') as csvFile:
    rocketWriter = csv.writer(csvFile)
    rocketWriter.writerow(('ini lat (deg)','ini lon (deg)','ini alt (m)','ini vx_body (m/s)','ini vy_body (m/s)','ini vz_body (m/s)','ini yaw (deg)','ini pitch (deg)','ini roll (deg)'))
    rocketWriter.writerow( ('32.990254', '-106.974998', '1400', v_b[0,0], v_b[0,1], v_b[0,2], ypr[0,0], ypr[0,1], ypr[0,2] ))
    rocketWriter.writerow(('command type','yaw (deg)','pitch (deg)','roll (deg)','vx_body (m/s)','vy_body (m/s)','vz_body (m/s)','command duration (s)','GPS visibility'))
    for idx in range(1,len(t)): 
        rocketWriter.writerow( ( '2', ypr[idx,0],ypr[idx,1], ypr[idx,2], v_b[idx,0], v_b[idx,1], v_b[idx,2], t[idx] - t[idx - 1], '1' ))
    


