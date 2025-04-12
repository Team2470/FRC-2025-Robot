package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelights extends SubsystemBase {

    public static enum Limelight {
        kLeft("limelight-left"),
        kMiddle("limelight-middle"),
        kRight("limelight-right");

        public final String name;

        private Limelight(String name) {
            this.name = name;
        }
    }


    public Limelights() {
        if (RobotBase.isSimulation()) {
            // Just make the limelight report it sees something
            LimelightHelpers.setLimelightNTDouble(Limelight.kLeft.name, "tv", 1);
            LimelightHelpers.setLimelightNTDouble(Limelight.kLeft.name, "tx", 0);
            LimelightHelpers.setLimelightNTDouble(Limelight.kLeft.name, "ty", 0);
            LimelightHelpers.setLimelightNTDouble(Limelight.kLeft.name, "tid", 11);

            // Just make the limelight report it sees something
            LimelightHelpers.setLimelightNTDouble(Limelight.kRight.name, "tv", 1);
            LimelightHelpers.setLimelightNTDouble(Limelight.kRight.name, "tx", 0);
            LimelightHelpers.setLimelightNTDouble(Limelight.kRight.name, "ty", 0);
            LimelightHelpers.setLimelightNTDouble(Limelight.kRight.name, "tid", 11);
        }
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            LimelightHelpers.SetIMUMode(Limelight.kLeft.name, 2);
            // Disable throttle
            LimelightHelpers.SetThrottle(Limelight.kLeft.name, 0);
            LimelightHelpers.SetThrottle(Limelight.kRight.name, 0);
        } else {
            LimelightHelpers.SetIMUMode(Limelight.kLeft.name, 1);
            // Enable throttle when disabled
            LimelightHelpers.SetThrottle(Limelight.kLeft.name, 10);
            LimelightHelpers.SetThrottle(Limelight.kRight.name, 10);
        }

        LimelightHelpers.setPipelineIndex(Limelight.kLeft.name, 2);
        LimelightHelpers.setPipelineIndex(Limelight.kRight.name, 2);


        SmartDashboard.putBoolean("Limelight left tv", LimelightHelpers.getTV(Limelight.kLeft.name));
        SmartDashboard.putBoolean("Limelight right tv", LimelightHelpers.getTV(Limelight.kRight.name));
    }


}
