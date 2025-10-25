package frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Degrees;

import java.util.concurrent.Flow.Processor;

import edu.wpi.first.units.measure.Angle;


public class SuperstructurePosition {
    public enum TargetAction {
        HOME(
            ElevatorPositions.DRIVE.height,
            ArmPositions.DRIVE.degrees,
            WristPivotPositions.DRIVE.degrees,
            ActionType.NONE
        ),
        HPSTART(
            ElevatorPositions.HP.height,
            ArmPositions.HPSTART.degrees,
            WristPivotPositions.HPINTAKE.degrees,
            ActionType.CORAL
        ),
        HPHOLD(
            ElevatorPositions.HP.height,
            ArmPositions.HPHOLD.degrees,
            WristPivotPositions.HPINTAKE.degrees,
            ActionType.CORAL
        ),
        L1(
            ElevatorPositions.L1.height,
            ArmPositions.L1.degrees,
            WristPivotPositions.L1.degrees,
            ActionType.CORAL
        ),
        L2(
            ElevatorPositions.L2.height,
            ArmPositions.L2.degrees,
            WristPivotPositions.L2.degrees,
            ActionType.CORAL
        ),
        L3(
            ElevatorPositions.L3.height,
            ArmPositions.L3.degrees,
            WristPivotPositions.L3.degrees,
            ActionType.CORAL
        ),
        L4(
            ElevatorPositions.L4.height,
            ArmPositions.L4.degrees,
            WristPivotPositions.L4.degrees,
            ActionType.CORAL
        ),
        GROUND_ALGAE(
            ElevatorPositions.GROUNDALGAE.height,
            ArmPositions.GROUNDALGAE.degrees,
            WristPivotPositions.GROUNDALGAE.degrees,
            ActionType.ALGAE
        ),
        LOWER_ALGAE(
            ElevatorPositions.LOWALGAE.height,
            ArmPositions.LOWALGAE.degrees,
            WristPivotPositions.LOWALGAE.degrees,
            ActionType.ALGAE
        ),
        UPPER_ALGAE(
            ElevatorPositions.HIGHALGAE.height,
            ArmPositions.HIGHALGAE.degrees,
            WristPivotPositions.HIGHALGAE.degrees,
            ActionType.ALGAE
        ),
        ALGAE_NET(
            ElevatorPositions.NET.height,
            ArmPositions.NET.degrees,
            WristPivotPositions.NET.degrees,
            ActionType.ALGAE
        ),
        ALGAE_PROCESSOR(
            ElevatorPositions.PROCESSOR.height,
            ArmPositions.PROCESSOR.degrees,
            WristPivotPositions.PROCESSOR.degrees,
            ActionType.ALGAE
        );

        // spotless:on
        private final double elevatorPosition;
        private final double armAngle;
        private final double wristAngle;
        private final ActionType type;

        private TargetAction(double elevatorPosition, double armAngle, double wristAngle, ActionType type) {
            this.elevatorPosition = elevatorPosition;
            this.armAngle = armAngle;
            this.wristAngle = wristAngle;
            this.type = type;
        }

        public double getElevatorPosition() {
            return elevatorPosition;
        }

        public double getArmAngle() {
            return armAngle;
        }

        public double getWristAngle() {
            return wristAngle;
        }

        public ActionType getType() {
            return type;
        }
    }

    public enum ActionType {
        NONE,
        CORAL,
        ALGAE
    }

    // TODO: Tune values for each position and make them constants
    // (IE: make a constants folder and replace numbers with constant names)
    private enum WristPivotPositions {
        DRIVE(85),
        HPINTAKE(180),
        GROUNDALGAE(85),
        LOWALGAE(-23),
        HIGHALGAE(-23),
        NET(46.0),
        PROCESSOR(125),
        L1(33),
        L2(131),
        L3(125),
        L4(125);

        private double degrees;

        private WristPivotPositions(double degrees) {
            this.degrees = degrees;
        }
    }

    private enum ArmPositions {
        DRIVE(82),
        HPSTART(45),
        HPHOLD(55),
        GROUNDALGAE(60),
        LOWALGAE(40),
        HIGHALGAE(40),
        NET(70),
        PROCESSOR(72),
        L1(78),
        L2(50),
        L3(60),
        L4(60);

        private double degrees;

        private ArmPositions(double degrees) {
            this.degrees = degrees;
        }
    };

    private enum ElevatorPositions {
        DRIVE(0.33),
        HP(3),
        GROUNDALGAE(8),
        LOWALGAE(31.5),
        HIGHALGAE(49),
        NET(60),
        PROCESSOR(3),
        L1(5),
        L2(14),
        L3(30),
        L4(54.77);

        private double height; // Inches

        private ElevatorPositions(double height) {
            this.height = height;
        }
    }
}
