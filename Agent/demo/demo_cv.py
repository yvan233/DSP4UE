import airsim
import math


"""
Settings 
{
	"SettingsVersion": 1.2,
	"Vehicles": {
		"cv": {
			"VehicleType": "ComputerVision",
			"X": 0, "Y": 0, "Z": -2,
			"Pitch": 0, "Roll": 0, "Yaw": 0
		}
	}
}
"""

def list2Pose(list):
    return airsim.Pose(airsim.Vector3r(list[0], list[1], list[2]), 
        airsim.to_quaternion(math.radians(list[3]), math.radians(list[4]), math.radians(list[5])))
client = airsim.VehicleClient()
client.confirmConnection()

# x,y,z,pitch,roll,yaw
pose1 = [20,30,-50,-30,0,-120]
pose2 = [20,-60,-50,-40,0,135]

airsim.wait_key('Press any key to switch pose 1')
client.simSetVehiclePose(list2Pose(pose1), True)

airsim.wait_key('Press any key to switch pose 2')
client.simSetVehiclePose(list2Pose(pose2), True)