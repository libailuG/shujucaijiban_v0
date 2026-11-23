"""No hardware IO; deterministic upright zero-gyro example."""
from controller import Controller
c=Controller()
for tick in range(15):
 status,torque=c.step([0,0],[0,0],[1,0,0,0],[0,0,0])
 print(tick,status,torque.tolist())
