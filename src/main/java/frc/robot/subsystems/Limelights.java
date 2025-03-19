package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        setDefaultCommand(enableThrottle().ignoringDisable(true));
    }

    public Command enableThrottle() {
        return Commands.run(
            () -> {
                // Enable throttle when disabled
                // LimelightHelpers.SetThrottle(Limelight.kLeft.name, 150);
                // LimelightHelpers.SetThrottle(Limelight.kRight.name, 150);
            },
            this
        ).withName("Enable Throttle");
    }

    public Command disableThrottle() {
        return Commands.run(
            () -> {
                // Disable throttle
                LimelightHelpers.SetThrottle(Limelight.kLeft.name, 0);
                LimelightHelpers.SetThrottle(Limelight.kRight.name, 0);
            },
            this
        ).withName("Disable Throttle");
    }

}
