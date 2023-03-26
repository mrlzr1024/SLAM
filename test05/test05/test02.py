

# Import DroneKit-Python
from dronekit import connect, VehicleMode
import time

vehicle = connect("/dev/myusb0",baud=115200, wait_ready=True)
aTargetAltitude=vehicle.location.global_relative_frame.alt
# Get some vehicle attributes (state)
print ("Get some vehicle attribute values:")
print ("GPS: %s" % vehicle.gps_0)
print ("Battery: %s" % vehicle.battery)
print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
print ("Is Armable?: %s" % vehicle.is_armable)
print ("System status: %s" % vehicle.system_status.state)
print ("Mode: %s" % vehicle.mode.name)    # settable


msg=vehicle.message_factory
try:
    while True:
        print(f"local={vehicle.location.local_frame}")
        time.sleep(0.5)
except:
    pass


vehicle.close()


print("Completed")
