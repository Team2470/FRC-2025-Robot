import ntcore
import time


print("Hello world")

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("encoderLog")
inst.setServer("localhost")
sd = inst.getTable("SmartDashboard")

with open("encoderLog.csv", "w") as f:

    f.write("arm absolute position,arm absolute position filtered,arm position\n")
    while True:
        f.write(f'{ sd.getNumber("Arm Absolute Position",-1)},{sd.getNumber("Arm Absolute Position Filtered",-1)},{sd.getNumber("Arm Position",-1)}\n',)
        time.sleep(0.01)