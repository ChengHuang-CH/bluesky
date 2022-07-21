import os, sys
import time
# if 'SUMO_HOME' in os.environ:
#     tools = os.path.join('/usr/share/sumo', 'tools')
#     sys.path.append(tools)
# else:
#     sys.exit("please declare environment variable 'SUMO_HOME'")

tools = os.path.join('/usr/share/sumo', 'tools')
sys.path.append(tools)
# import libsumo as traci
import traci

sumoBinary = "sumo"  # no-gui
# sumoBinary = "sumo-gui"  # with-gui
sumoCmd = [sumoBinary, "-c", "/home/cheng/Documents/0_PythonFiles/opendrive/2022-07-21-20-48-44/osm.sumocfg",
           "--start"]

# sumoCmd.extend(["--eager-insert", "true", "--no-warnings", "--ignore-route-errors",
#                          "--collision.action", "none", "--no-step-log", "--step-length", "0.1"])

traci.start(sumoCmd)
step = 0
while step < 1000:
    time.sleep(0.1)
    traci.simulationStep()
    print('------------------')
    # print(traci.vehicle.getIDCount())
    # print(traci.vehicle.getIDList())
    ids = traci.vehicle.getIDList()
    vehID = ids[0]
    x, y = traci.vehicle.getPosition(vehID)
    lon, lat = traci.simulation.convertGeo(x, y)
    speed = traci.vehicle.getSpeed(vehID)
    print(f'{vehID}: lon-lat{lon, lat} x-y:{x, y} speed: {speed}')

    step += 1
    print(step)


traci.close()
