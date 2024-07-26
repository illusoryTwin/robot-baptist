import mujoco
import mujoco.viewer
from mujoco import minimize
import time
import numpy as np

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)
# Initialize lists to store the data
q_t = []
dq_t = []
ddq_t = []
torques = []
end_time = 30


# Open the file and read lines
# with open('trajectory_data.txt', 'r') as file:
with open('trajectory_data.csv', 'r') as file:
    lines = file.readlines()

# Process each line
for line in lines:
    # Split the line into individual values and convert them to floats
    values = list(map(float, line.strip().split(',')))

    # Append the values to q_t, dq_t, ddq_t
    q_t.append(values[:4])
    dq_t.append(values[4:8])
    ddq_t.append(values[8:12])
    torques.append(values[12:16])


# T = 1.875
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()
        data.qpos[:] = np.deg2rad([5, 5, 5, 5])
        time_ = 0
        # while time_ <= T:
        for i in range(len(q_t)):
            data.qpos[:] = q_t[i]
            data.qvel[:] = dq_t[i]
            data.qacc[:] = ddq_t[i]
            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)