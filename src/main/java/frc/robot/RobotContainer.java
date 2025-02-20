// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.*;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                 // angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController controller = new CommandXboxController(0);
  
  private final CommandJoystick buttonPad = new CommandJoystick(1);
  private final CommandJoystick testButtonPad = new CommandJoystick(2);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator1 = new Elevator();
  private final Wrist wrist = new Wrist();
  private final Arm arm = new Arm();
  private final OuterIntake algea = new OuterIntake(0, 43, false);
  private final InnerIntake coral = new InnerIntake(41, 44, false);


  // Autos
  private final RevDigit m_revDigit;
  private final AutoSelector m_autoSelector;

  public RobotContainer() {

    // Auto Selector
    m_revDigit = new RevDigit().display("2470");

    m_autoSelector = new AutoSelector(
        m_revDigit, "DFLT", new PrintCommand("!!! Invalid Auto Selected !!!"));


    NamedCommands.registerCommands(new HashMap<String, Command>() {
      {
        // put("speaker-shoot", speakerShoot());
        put("L2", autoReefCommand());
        put("HoldL2", reefL2Command());
        put("OuttakeCoral", runInTakeCommand(-8).withTimeout(1).withName("Auto Run Outtake"));
        put("OuttakeCoral", runInTakeCommand(8).withTimeout(1).withName("Auto Run Intkae"));
        put("DrivePos", autoDrivePositiCommand());
        put("L1", reefL1Command());
      }
    });

    // registerAutos(new HashMap<String, String>() {
    //   {

    //     put("Foo", "foo");
    //   }
    // });
    m_autoSelector.registerCommand("FOO", "FOO", AutoBuilder.buildAuto("Foo"));
    m_autoSelector.registerCommand("STRT", "STRT", AutoBuilder.buildAuto("STRT"));
    m_autoSelector.registerCommand("NOBK", "NOBK", AutoBuilder.buildAuto("NOBK"));
    m_autoSelector.registerCommand("R1", "R1", AutoBuilder.buildAuto("R1"));
    m_autoSelector.registerCommand("R2", "R2", AutoBuilder.buildAuto("R2"));
    m_autoSelector.registerCommand("Trsh", "Trsh", AutoBuilder.buildAuto("Trsh"));
    m_autoSelector.registerCommand("TRH2", "TRH2", AutoBuilder.buildAuto("TRH2"));


    configureBindings();
    m_autoSelector.initialize();

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Drive", drivetrain);
    SmartDashboard.putData("Elevator", elevator1);
    SmartDashboard.putData("Arm", arm);
    SmartDashboard.putData("Wrist", wrist);
    SmartDashboard.putData("InnerIntake", coral);
    SmartDashboard.putData("OuterIntake", algea);

    SmartDashboard.putNumber("MaxAngularRate", MaxAngularRate);
  }

  private void configureBindings() {

    //
    // Drive Train
    //

    final var xFilter = new SlewRateLimiter(5);
    final var yFilter = new SlewRateLimiter(5);
    final var rotateFilter = new SlewRateLimiter(5);

    BooleanSupplier slowModeSupplier = () -> controller.getHID().getXButton();

    DoubleSupplier rotationSupplier = () -> {
      double leftTrigger = controller.getHID().getLeftTriggerAxis();
      double rightTrigger = controller.getHID().getRightTriggerAxis();

      SmartDashboard.putNumber("Left Trigger", leftTrigger);
      SmartDashboard.putNumber("Right Trigger", rightTrigger);

      double rotate = 0.0;
      if (leftTrigger < rightTrigger) {
        rotate = -rightTrigger;
      } else {
        rotate = leftTrigger;
      }

      rotate = Math.copySign(rotate*rotate, rotate);
      rotate = rotateFilter.calculate(rotate) * MaxAngularRate;
      SmartDashboard.putNumber("Rotate", rotate);
      return rotate;
    };

    Supplier<Optional<Translation2d>> translationSupplier = () -> {
      // Read gamepad joystick state, and apply slew rate limiters

      // X Move Velocity - Forward
      double xMove = MathUtil.applyDeadband(xFilter.calculate(-controller.getHID().getLeftY()), .05);
      
      // Y Move Velocity - Strafe
      double yMove = MathUtil.applyDeadband(yFilter.calculate(-controller.getHID().getLeftX()), .05);

      if (controller.getHID().getLeftBumperButton()) {
        yMove = 0;
      }

      if (xMove == 0 && yMove == 0) {
        return Optional.empty();
      }

      Translation2d move = new Translation2d(xMove, yMove);

      return Optional.of(new Translation2d(
          // Scale the speed of the robot by using a quadratic input curve.
          // and convert the joystick values -1.0-1.0 to Meters Per Second
          Math.pow(move.getNorm(), 2) * MaxSpeed,
          // Get the direction the joystick is pointing
          move.getAngle()
      ));
    };

    // Field-centric by default
    final var fieldCentric = new SwerveRequest.FieldCentric()
      // .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    final var fieldCentricIdle = new SwerveRequest.Idle();
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          if (DriverStation.isAutonomousEnabled()) {
            return fieldCentricIdle;
          }

          var translation = translationSupplier.get();

          double rotate = rotationSupplier.getAsDouble();
          double xMove = 0;
          double yMove = 0;
        
          if (translation.isPresent()) {
            xMove = translation.get().getX();
            yMove = translation.get().getY();
          }
    
          if (slowModeSupplier.getAsBoolean()) {
            xMove *= 0.5;
            yMove *= 0.5;
            rotate *= 0.25;
          } else {
            xMove *= 1.0;
            yMove *= 1.0;
            rotate *= 0.5;
          }

          return fieldCentric
              .withVelocityX(xMove)
              .withVelocityY(yMove)
              .withRotationalRate(rotate);
        }).withName("Field Centric with GamePad"));

    // Robot Centric when pressing A
    final var robotCentric = new SwerveRequest.RobotCentric();

    // joystick.leftBumper().whileTrue(elevatorUpCommand());
    // joystick.rightBumper().whileTrue(elevatorDownCommand());
    // joystick.leftBumper().whileTrue(elevatorToPostitonCommandDash(75));
    // joystick.rightBumper().whileTrue(elevatorToPostitonCommandDash(35));
    // joystick.y().whileTrue(elevatorToPostitonCommandDash());

    controller.a().whileTrue(drivetrain.applyRequest(() -> {
      var translation = translationSupplier.get();

      double rotate = rotationSupplier.getAsDouble();
      double xMove = 0;
      double yMove = 0;

      if (translation.isPresent()) {
        xMove = translation.get().getX();
        yMove = translation.get().getY();
      }

      if (slowModeSupplier.getAsBoolean()) {
        xMove *= 0.5;
        yMove *= 0.5;
        rotate *= 0.25;
      } else {
        xMove *= 1.0;
        yMove *= 1.0;
        rotate *= 0.5;
      }

      return robotCentric
          .withVelocityX(xMove)
          .withVelocityY(yMove)
          .withRotationalRate(rotate);
    }).withName("Robot Centric with GamePad"));

    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    controller.rightStick().toggleOnTrue(drivetrain.applyRequest(() -> brake));

    // Reset the field-centric heading on start press
    controller.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Reset robot pose to 0,0, and 0 degrees
    controller.back().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d())));

    drivetrain.registerTelemetry(logger::telemeterize);


    testButtonPad.button(9).whileTrue(elevator1.elevatorHomeCommand());
    // controller.leftBumper().whileTrue(new ParallelCommandGroup(algea.runMotorForwardsSpeedCommand(3), coral.runMotorForwardsSpeedCommand(3)));
    // controller.rightBumper().whileTrue(new ParallelCommandGroup(algea.runMotorBackwardsSpeedCommand(8), coral.runMotorBackwardsSpeedCommand(8)));
    // controller.leftBumper().whileTrue(new SequentialCommandGroup(
    //   arm.pidCommand(15).until(()-> Math.abs(arm.getErrorAngle()) < 3),
    //   arm.coastCommand()
    // ));


    
    buttonPad.button(9).whileTrue(new SequentialCommandGroup(
      
      arm.pidCommand(85).until(()-> Math.abs(arm.getErrorAngle()) < 3),
      new ParallelCommandGroup(
        arm.pidCommand(85),
        wrist.pidCommand(85)
      )



    ));

    buttonPad.button(9).whileTrue(pickupCommand());
    buttonPad.button(5).whileTrue(pickupAlgaeCommand());
    buttonPad.button(3).and(buttonPad.button(2)).whileTrue(elevator1.openLoopCommand(2));
    buttonPad.button(3).and(buttonPad.button(6)).whileTrue(elevator1.openLoopCommand(-2));
    buttonPad.button(7).and(buttonPad.button(2)).whileTrue(arm.openLoopCommand(2));
    buttonPad.button(7).and(buttonPad.button(6)).whileTrue(arm.openLoopCommand(-2));
    buttonPad.button(11).and(buttonPad.button(2)).whileTrue(wrist.openLoopCommand(1));
    buttonPad.button(11).and(buttonPad.button(6)).whileTrue(wrist.openLoopCommand(-1));
    buttonPad.button(4).whileTrue(runInTakeCommand(4));
    buttonPad.button(8).whileTrue(runInTakeCommand(-4));
    buttonPad.button(10).whileTrue(reefL2Command());
    buttonPad.button(11).whileTrue(reefL3Command());
    buttonPad.button(12).whileTrue(reefL4Command());
    buttonPad.button(1).whileTrue(reefL1Command());





    
    arm.setDefaultCommand(drivePositiCommand());


    // wrist.setDefaultCommand(new SequentialCommandGroup(
    //   new ConditionalCommand(
    //     wrist.pidCommand(85).until(()-> Math.abs(wrist.getPosition() - 85) < 3), 
    //     wrist.pidCommand(85).until(()-> Math.abs(wrist.getPosition() - 85) < 3), 
    //     () -> arm.getPosition() < 30
    //   ),
    //   // wrist.pidCommand(85).until(()-> wrist.getPosition() > 80),
    //   new WaitUntilCommand(()-> Math.abs(arm.getPosition() - 85) < 3),
    //   new ParallelCommandGroup(
    //     // arm.pidCommand(85),
    //     // wrist.pidCommand(85),
    //     // new WaitUntilCommand(()-> Math.abs(elevator1.getErrorPercent()) < 2)
    //     new RunCommand(()->{}))
    // )));

    // arm.setDefaultCommand(new SequentialCommandGroup(
    //   new ConditionalCommand(
    //     new WaitUntilCommand(()-> Math.abs(wrist.getPosition() - 85) < 3), 
    //     new WaitUntilCommand(()-> Math.abs(wrist.getPosition() - 85) < 3), 
    //     () -> arm.getPosition() < 30
    //   ),
    //   // wrist.pidCommand(85).until(()-> wrist.getPosition() > 80),
    //   arm.pidCommand(85).until(()-> Math.abs(arm.getPosition() - 85) < 3),
    //   new ParallelCommandGroup(
    //     arm.pidCommand(85),
    //     // wrist.pidCommand(85),
    //     // elevator1.pidCommand(1).until(()-> Math.abs(elevator1.getErrorPercent()) < 2)
    // )));
    // elevator1.setDefaultCommand(new SequentialCommandGroup(
    //   new ConditionalCommand(
    //     new WaitUntilCommand(()-> Math.abs(wrist.getPosition() - 85) < 3), 
    //     new WaitUntilCommand(()-> Math.abs(wrist.getPosition() - 85) < 3), 
    //     () -> arm.getPosition() < 30
    //   ),
    //   // wrist.pidCommand(85).until(()-> wrist.getPosition() > 80),
    //   new WaitUntilCommand(()-> Math.abs(arm.getPosition() - 85) < 3),
    //   new ParallelCommandGroup(
    //     // arm.pidCommand(85),
    //     // wrist.pidCommand(85),
    //     elevator1.pidCommand(1).until(()-> Math.abs(elevator1.getErrorPercent()) < 2),
    //     new RunCommand(()->{}))
    // )));
    testButtonPad.button(1).whileTrue(elevator1.openLoopCommand(2));
    testButtonPad.button(5).whileTrue(elevator1.openLoopCommand(-2));
    testButtonPad.button(10).whileTrue(elevator1.pidCommand(0.5));
    testButtonPad.button(6).whileTrue(elevator1.pidCommand(24));
    testButtonPad.button(2).whileTrue(elevator1.pidCommand(48));


    testButtonPad.button(3).whileTrue(arm.openLoopCommand(1));
    testButtonPad.button(7).whileTrue(arm.openLoopCommand(-1));
    testButtonPad.button(11).whileTrue(arm.pidCommand(20));
    testButtonPad.button(12).whileTrue(arm.pidCommand(65));
    testButtonPad.button(4).whileTrue(wrist.pidCommand(90));
    testButtonPad.button(8).whileTrue(wrist.pidCommand(0));
    
  }

  public Command getAutonomousCommand() {
    return m_autoSelector.selected();
  }

  private Command drivePositiCommand(){
    return new SequentialCommandGroup(
      new ConditionalCommand(
        wrist.pidCommand(85).until(()-> Math.abs(wrist.getPosition() - 85) < 10), 
        wrist.pidCommand(85).until(()-> Math.abs(wrist.getPosition() - 85) < 10), 
        () -> arm.getPosition() < 30
      ),
      // wrist.pidCommand(85).until(()-> wrist.getPosition() > 80),
      arm.pidCommand(77).until(()-> Math.abs(arm.getPosition() - 77) < 3),
      new ParallelCommandGroup(
        arm.pidCommand(77),
        wrist.pidCommand(85),
        new ScheduleCommand(elevator1.pidCommand(1).until(()-> Math.abs(elevator1.getErrorPercent()) < 2))

    )).withName("TeleOp Drive Position");
  }


  
  private Command autoDrivePositiCommand(){
    return new SequentialCommandGroup(
      new ConditionalCommand(
        wrist.pidCommand(85).until(()-> Math.abs(wrist.getPosition() - 85) < 10), 
        wrist.pidCommand(85).until(()-> Math.abs(wrist.getPosition() - 85) < 10), 
        () -> arm.getPosition() < 30
      ),
      // wrist.pidCommand(85).until(()-> wrist.getPosition() > 80),
      arm.pidCommand(77).until(()-> Math.abs(arm.getPosition() - 77) < 3),
      new ParallelCommandGroup(
        arm.pidCommand(77),
        wrist.pidCommand(85),
        elevator1.pidCommand(1).until(()-> Math.abs(elevator1.getErrorPercent()) < 2)

    )).withName("Auto Drive Position");
  }
  private Command pickupCommand(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        arm.pidCommand(60),
        wrist.pidCommand(85)
      ).until(()-> Math.abs(arm.getPosition() - 60) < 5),
      new ParallelCommandGroup(
        wrist.pidCommand(0),
        arm.coastCommand(),
        new ParallelCommandGroup(
          algea.runMotorBackwardsSpeedCommand(8), 
          coral.runMotorBackwardsSpeedCommand(4)
        ).until(coral::haveCoral)
      )
    );
  }

  private Command runInTakeCommand(int voltage) {
    return new ParallelCommandGroup(
      algea.runMotorForwardsSpeedCommand(2 * voltage), coral.runMotorForwardsSpeedCommand(voltage)
    );

  }

  private Command reefL2Command(){
    return new SequentialCommandGroup(
      arm.pidCommand(60).until(()-> Math.abs(arm.getErrorAngle()) < 3),

      new ParallelCommandGroup(
        elevator1.pidCommand(17),
        arm.pidCommand(60)
      ).until(()->Math.abs(elevator1.getErrorPercent()) < 2),
    new ParallelCommandGroup(
      elevator1.pidCommand(17),
      arm.pidCommand(60),
      wrist.pidCommand(125)
    )
    ).withName("L2 Reef Command");
  }

  private Command reefL3Command(){
    return new SequentialCommandGroup(
      arm.pidCommand(60).until(()-> Math.abs(arm.getErrorAngle()) < 3),

      new ParallelCommandGroup(
        elevator1.pidCommand(30),
        arm.pidCommand(60)
      ).until(()->Math.abs(elevator1.getErrorPercent()) < 2),
    new ParallelCommandGroup(
      elevator1.pidCommand(30),
      arm.pidCommand(60),
      wrist.pidCommand(125)
    )
    );
  }

  private Command reefL4Command(){
    return new SequentialCommandGroup(
      arm.pidCommand(60).until(()-> Math.abs(arm.getErrorAngle()) < 3),

      new ParallelCommandGroup(
        elevator1.pidCommand(54.77),
        arm.pidCommand(60)
      ).until(()->Math.abs(elevator1.getPosition()- 54.77) < 2),
    new ParallelCommandGroup(
      elevator1.pidCommand(54.77),
      arm.pidCommand(60),
      wrist.pidCommand(125)
    )
    );
  }

  private Command autoReefCommand(){
    return new SequentialCommandGroup(
      reefL2Command().until(()-> Math.abs(elevator1.getErrorPercent()) < 3)
    ).withName("Auto Reef Command");
  }

  private Command pickupAlgaeCommand(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
      arm.pidCommand(120),
      wrist.pidCommand(60)
      ).until(()-> Math.abs(arm.getErrorAngle()) < 5),
      new ParallelCommandGroup(
      wrist.pidCommand(20),
      arm.coastCommand(),
      new ParallelCommandGroup(algea.runMotorBackwardsSpeedCommand(8), coral.runMotorBackwardsSpeedCommand(4))
      )
    );
  }

  private Command reefL1Command(){
    return new SequentialCommandGroup(
      arm.pidCommand(78).until(()-> Math.abs(arm.getErrorAngle()) < 3),

      new ParallelCommandGroup(
        elevator1.pidCommand(5),
        arm.pidCommand(78)
      ).until(()->Math.abs(elevator1.getErrorPercent()) < 2),
    new ParallelCommandGroup(
      elevator1.pidCommand(5),
      arm.pidCommand(78),
      wrist.pidCommand(33)
    )
    ).withName("L1 Reef Command");
  }




}
