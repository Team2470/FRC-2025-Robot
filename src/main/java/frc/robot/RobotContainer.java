// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.Thread.State;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
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

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.m_State;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Aligntoreef;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraightBack;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  public static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                 // angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController controller = new CommandXboxController(0);

  private final CommandJoystick buttonPad = new CommandJoystick(1);
  private final CommandJoystick testButtonPad = new CommandJoystick(2);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Wrist wrist = new Wrist();
  private final OuterIntake algea = new OuterIntake(42, 44, false);
  private final InnerIntake coral = new InnerIntake(41, 43, false);
  private final HumanIntake intake = new HumanIntake(45, 46, true);
  private final Elevator elevator1 = new Elevator(intake.getCANDI());
  private final Arm arm = new Arm(algea.getCANDI());
  private final IntakeServo intakeServoRight = new IntakeServo(0, false);
  private final IntakeServo intakeServoLeft = new IntakeServo(2, true);
  private final Superstructure superstructure = new Superstructure(arm, elevator1, wrist);
  private final DigitalInput m_brakeButton = new DigitalInput(6);
  private final AddressableLED m_brakeLed = new AddressableLED(5);
  private final AddressableLEDBuffer m_brakeLedBuffer = new AddressableLEDBuffer(60);
  private final Climber m_Climber = new Climber(47, 3);
  private final Limelights m_limelights = new Limelights();
  // private final Climber m_climber = new Climber(, 9);

  private final Vision vision = new Vision();

  // Autos
  private final RevDigit m_revDigit;
  private final AutoSelector m_autoSelector;
  private final SwerveRequest m_idleRequest = new SwerveRequest.Idle(); 

  public RobotContainer() {

    m_brakeLed.setLength(m_brakeLedBuffer.getLength());
    m_brakeLed.setData(m_brakeLedBuffer);
    m_brakeLed.start();
    // Auto Selector
    m_revDigit = new RevDigit().display("2470");

    m_autoSelector = new AutoSelector(
        m_revDigit, "DFLT", new PrintCommand("!!! Invalid Auto Selected !!!"));

    NamedCommands.registerCommands(new HashMap<String, Command>() {
      {
        // put("speaker-shoot", speakerShoot());
        put("L2", autoReefCommand());
        put("HoldL2", reefL2Command());
        put("OuttakeCoral", new SequentialCommandGroup(runInTakeCommand(-12).until(()-> !coral.haveCoral()),
        runInTakeCommand(-12).withTimeout(0.4)
        ).withName("Auto Run Outtake"));
        // put("OuttakeCoral", runInTakeCommand(8).withTimeout(1).withName("Auto Run Intkae"));
        put("DrivePos", drivePositiCommand());
        put("L1", reefL1Command());
        put("L2", reefL2Command());
        put("L3", reefL3Command());
        put("L4", reefL4Command());
        put("0.5W-DrivePos", new SequentialCommandGroup(new WaitCommand(0.5), drivePositiCommand()).until(()-> elevator1.getPosition() < 5));
        put("HpIntake", new SequentialCommandGroup(
          drivePositiCommand().until(()-> elevator1.getPosition() < 2),
          HumanPlayerIntakeCommand().until(coral::haveCoral))
        
        );
        put("Align Left", Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Left, Aligntoreef.Score.Coral, "Auto Align left").withTimeout(3));
        put("Align Right", Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Right, Aligntoreef.Score.Coral, "Auto Align left").withTimeout(3));
        put("Align Left2", Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Left, Aligntoreef.Score.Coral, "Auto Align left").withTimeout(3));

        put("DSLR2", new SequentialCommandGroup(new WaitUntilCommand(()-> elevator1.getPosition() > 30), new DriveStraight(drivetrain, 0.125).withName("Drive straigt left reef")));

        put("DSLR", new SequentialCommandGroup(new WaitUntilCommand(()-> elevator1.getPosition() > 30), new DriveStraight(drivetrain, 0.145).withName("Drive straigt left reef")));
        put("DSRR", new DriveStraight(drivetrain, 0.085).withName("Drive straight right reef"));
        put("DSBack", new DriveStraightBack(drivetrain, 0.22).withName("Drive straight backwards"));
        // put("ResVis", setVisionPose());
        // put("drive straight right reef", new DriveStraight(drivetrain, 0.218));
        // put("drive straight right reef", new DriveStraightBack(drivetrain, 0.2));

        put("stop", drivetrain.applyRequest(() -> m_idleRequest).withName("stop"));
        put("debug-false", Commands.runOnce(() -> SmartDashboard.putBoolean("Auto debug flag", false)));
        put("debug-true", Commands.runOnce(() -> SmartDashboard.putBoolean("Auto debug flag", true)));

        // put("scoreL4LeftCoral", new ParallelDeadlineGroup(
        //   new SequentialCommandGroup(
        //     new WaitUntilCommand(() -> {
        //       return Math.abs(elevator1.getPosition() - 54.77 ) < 1  && Math.abs(arm.getPosition() - 60) < 2 && Math.abs(wrist.getPosition() - 125) < 2;
        //     },
        //     new DriveStraight(drivetrain, 0.24),
        //     runInTakeCommand(-8).until(()-> !coral.haveCoral()).withName("Auto Run Outtake"),

        //   ),
        //   reefL4Command()
        // ));
      }
    });

    // registerAutos(new HashMap<String, String>() {
    // {

    // put("Foo", "foo");
    // }
    // });
    // m_autoSelector.registerCommand("FOO", "FOO", AutoBuilder.buildAuto("Foo"));
    // m_autoSelector.registerCommand("STRT", "STRT", AutoBuilder.buildAuto("STRT"));
    // m_autoSelector.registerCommand("NOBK", "NOBK", AutoBuilder.buildAuto("NOBK"));
    // m_autoSelector.registerCommand("R1", "R1", AutoBuilder.buildAuto("R1"));
    // m_autoSelector.registerCommand("R2", "R2", AutoBuilder.buildAuto("R2"));
    // m_autoSelector.registerCommand("Trsh", "Trsh", AutoBuilder.buildAuto("Trsh"));
    // m_autoSelector.registerCommand("TRH2", "TRH2", AutoBuilder.buildAuto("TRH2"));
      m_autoSelector.registerCommand("MG", "MGMG", AutoBuilder.buildAuto("MG"));
      m_autoSelector.registerCommand("LIKL", "LIKL", AutoBuilder.buildAuto("LIKL"));
      m_autoSelector.registerCommand("032025 push", "PUSH", AutoBuilder.buildAuto("032025 push"));
      m_autoSelector.registerCommand("Vision Mg", "vMG", AutoBuilder.buildAuto("Vision Mg"));

      
    configureBindings();
    m_autoSelector.initialize();
    SmartDashboard.putData("AutoSelector next auto", m_autoSelector.nextAutoCommand());
    SmartDashboard.putData("AutoSelector previous auto", m_autoSelector.previousAutoCommand());

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Drive", drivetrain);
    SmartDashboard.putData("Elevator", elevator1);
    SmartDashboard.putData("Arm", arm);
    SmartDashboard.putData("Wrist", wrist);
    SmartDashboard.putData("InnerIntake", coral);
    SmartDashboard.putData("OuterIntake", algea);
    SmartDashboard.putData("Limelights", m_limelights);
    SmartDashboard.putData("Vision", vision);
    SmartDashboard.putNumber("MaxAngularRate", MaxAngularRate);

  }

  private void configureBindings() {

    //
    // Drive Train
    //

    final var xFilter = new SlewRateLimiter(5);
    final var yFilter = new SlewRateLimiter(5);
    final var rotateFilter = new SlewRateLimiter(5);

    DoubleSupplier slowModeSupplier = () -> {

      if (elevator1.getPosition() > 50 ) {
        return 0.25;
      }

      if (elevator1.getPosition() > 10) {
        return 0.5;
      }

      // No slow mode
      return 1.0;
    };

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

      rotate = Math.copySign(rotate * rotate, rotate);
      rotate = rotateFilter.calculate(rotate) * MaxAngularRate;
      SmartDashboard.putNumber("Rotate", rotate);
      return rotate;
    };

    Supplier<Optional<Translation2d>> translationSupplier = () -> {
      // Read gamepad joystick state, and apply slew rate limiters

      // X Move Velocity - Forward
      double xMove = xFilter.calculate(-controller.getHID().getLeftY());

      // Y Move Velocity - Strafe
      double yMove = yFilter.calculate(-controller.getHID().getLeftX());

      // if (controller.getHID().getLeftBumperButton()) {
      // yMove = 0;
      // }

      if (xMove == 0 && yMove == 0) {
        return Optional.empty();
      }

      Translation2d move = new Translation2d(xMove, yMove);

      return Optional.of(new Translation2d(
          // Scale the speed of the robot by using a quadratic input curve.
          // and convert the joystick values -1.0-1.0 to Meters Per Second
          Math.pow(move.getNorm(), 2) * MaxSpeed,
          // Get the direction the joystick is pointing
          move.getAngle()));
    };

    // Field-centric by default
    final var fieldCentric = new SwerveRequest.FieldCentric()
        // .withDeadband(MaxSpeed * 0.1)
        // .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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

          if (Math.abs(rotate) < MaxAngularRate * 0.05 && Math.abs(xMove) < MaxSpeed * 0.05 && Math.abs(yMove) < MaxSpeed * 0.05) {
            return fieldCentricIdle;
          }
          
          // option 1
          if (elevator1.getPosition() > 50) {
            xMove *= 0.5;
              yMove *= 0.5;
              rotate *= 0.25;
          } else if (elevator1.getPosition() > 10) {
            xMove *= 0.3;
            yMove *= 0.3;
            rotate *= 0.175;
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
        // yMove = translation.get().getY();
      }

      if (elevator1.getPosition() > 50) {
        xMove *= 0.5;
          yMove *= 0.5;
          rotate *= 0.25;
      } else if (elevator1.getPosition() > 10) {
        xMove *= 0.3;
        yMove *= 0.3;
        rotate *= 0.175;
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

    final SwerveRequest.FieldCentricFacingAngle leftHuman = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(5, 0, 0)
        .withTargetDirection(Rotation2d.fromDegrees(-54))
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    // Robot aligns with left human player station
    controller.leftBumper().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          var translation = translationSupplier.get();

          double xMove = 0;
          double yMove = 0;

          if (translation.isPresent()) {
            xMove = translation.get().getX();
            yMove = translation.get().getY();
          }
          if (elevator1.getPosition() > 50) {
            xMove *= 0.5;
              yMove *= 0.5;
          } else if (elevator1.getPosition() > 10) {
            xMove *= 0.3;
            yMove *= 0.3;
          } else {
            xMove *= 1.0;
            yMove *= 1.0;
          }

          return leftHuman
              .withVelocityX(xMove)
              .withVelocityY(yMove);
        }).withName("leftHuman"));

    // Robot aligns with right human player station
    final SwerveRequest.FieldCentricFacingAngle rightHuman = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(5, 0, 0)
        .withTargetDirection(Rotation2d.fromDegrees(54))
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    controller.rightBumper().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          var translation = translationSupplier.get();

          double xMove = 0;
          double yMove = 0;

          if (elevator1.getPosition() > 50) {
            xMove *= 0.5;
              yMove *= 0.5;
          } else if (elevator1.getPosition() > 10) {
            xMove *= 0.3;
            yMove *= 0.3;
          } else {
            xMove *= 1.0;
            yMove *= 1.0;
          }

          return rightHuman
              .withVelocityX(xMove)
              .withVelocityY(yMove);
        }).withName("rightHuman"));

    // Align to 180
    final SwerveRequest.FieldCentricFacingAngle alignNet = new SwerveRequest.FieldCentricFacingAngle()
    .withHeadingPID(5, 0, 0)
    .withTargetDirection(Rotation2d.fromDegrees(180))
    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        controller.y().whileTrue(processorCommand());
    // controller.y().whileTrue(netCommand());
// controller.y().whileTrue( // Drivetrain will execute this command periodically
//     drivetrain.applyRequest(() -> {
//       var translation = translationSupplier.get();

//       double xMove = 0;
//       double yMove = 0;

//       if (translation.isPresent()) {
//         xMove = translation.get().getX();
//         yMove = translation.get().getY();
//       }

//       if (slowModeSupplier.getAsBoolean()) {
//         xMove *= 0.5;
//         yMove *= 0.5;

//       } else {
//         xMove *= 1.0;
//         yMove *= 1.0;

//       }

//       return alignNet
//           .withVelocityX(xMove)
//           .withVelocityY(yMove);
//     }).withName("alignNet"));



    // Reset the field-centric heading on start press
    controller.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Reset robot pose to 0,0, and 0 degrees
    //controller.back().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d())));
    controller.back().onTrue(elevator1.elevatorHomeCommand());

    drivetrain.registerTelemetry(logger::telemeterize);

    testButtonPad.button(9).whileTrue(elevator1.elevatorHomeCommand());
    // controller.leftBumper().whileTrue(new
    // ParallelCommandGroup(algea.runMotorForwardsSpeedCommand(3),
    // coral.runMotorForwardsSpeedCommand(3)));
    // controller.rightBumper().whileTrue(new
    // ParallelCommandGroup(algea.runMotorBackwardsSpeedCommand(8),
    // coral.runMotorBackwardsSpeedCommand(8)));
    // controller.leftBumper().whileTrue(new SequentialCommandGroup(
    // arm.pidCommand(15).until(()-> Math.abs(arm.getErrorAngle()) < 3),
    // arm.coastCommand()
    // ));

    buttonPad.button(9).whileTrue(new SequentialCommandGroup(

        arm.pidCommand(85).until(() -> Math.abs(arm.getErrorAngle()) < 3),
        new ParallelCommandGroup(
            arm.pidCommand(85),
            wrist.pidCommand(85))

    ));

    buttonPad.button(9).whileTrue(pickupCommand());
    // buttonPad.button(5).whileTrue(pickupAlgaeCommand());
    buttonPad.button(5).whileTrue(HumanPlayerIntakeCommand());

    buttonPad.button(6).whileTrue(aLgaeL2Command());
    buttonPad.button(2).whileTrue(aLgaeL3Command());
    buttonPad.button(7).whileTrue(pickupAlgaeCommand());
    // buttonPad.button(7).whileTrue(());
    // buttonPad.button(3).and(buttonPad.button(2)).whileTrue(elevator1.openLoopCommand(2));
    // buttonPad.button(3).and(buttonPad.button(6)).whileTrue(elevator1.openLoopCommand(-2));
    // buttonPad.button(7).and(buttonPad.button(2)).whileTrue(arm.openLoopCommand(2));
    // buttonPad.button(7).and(buttonPad.button(6)).whileTrue(arm.openLoopCommand(-2));
    // buttonPad.button(11).and(buttonPad.button(2)).whileTrue(wrist.openLoopCommand(1));
    // buttonPad.button(11).and(buttonPad.button(6)).whileTrue(wrist.openLoopCommand(-1));
    buttonPad.button(4).whileTrue(runInTakeCommand(6));
    buttonPad.button(8).whileTrue(runInTakeCommand(-6));
    buttonPad.button(1).and(controller.x().negate()).and(controller.b().negate()).whileTrue(reefL1Command());
    buttonPad.button(10).and(controller.x().negate()).and(controller.b().negate()).whileTrue(reefL2Command());
    buttonPad.button(11).and(controller.x().negate()).and(controller.b().negate()).whileTrue(reefL3Command());
    buttonPad.button(12).and(controller.x().negate()).and(controller.b().negate()).whileTrue(reefL4Command());

    // buttonPad.button(10).whileTrue(reefL2Command());
    // buttonPad.button(11).whileTrue(reefL3Command());
    // buttonPad.button(12).whileTrue(reefL4Command());
    // buttonPad.button(1).whileTrue(reefL1Command());
    buttonPad.button(3).whileTrue(netCommand());
    controller.povUp().whileTrue(m_Climber.extendCommand());
    controller.povRight().whileTrue(testUndropIntake());
    controller.povDown().whileTrue(m_Climber.retractCommand());
    controller.povLeft().whileTrue(autoClimbCommand());
    // controller.povLeft().whileTrue(new SequentialCommandGroup(
    //     // new WaitUntilCommand(()-> m_Climber.getPosition() < 0.3),
    //     dropServoCommand())
    //   );
    // controller.povRight().whileTrue(dropServoCommand());

    // controller.x().whileTrue(new SequentialCommandGroup(
    // //new AddVisionMeasurement(drivetrain, vision).withTimeout(0.2),
    // new ParallelCommandGroup(
    //   new DeferredCommand(
    //     ()-> drivetrain.getAlignRightReef(), 
    //     Set.of(drivetrain)
    //   )
    //   // Commands.run(() -> {}, vision)
    // )).withName("Align Right Reef"));
    testButtonPad.button(9).whileTrue(new DriveStraightBack(drivetrain, 0.23));
    testButtonPad.button(1).whileTrue(new DriveStraight(drivetrain, 0.24));





      controller.x().whileTrue(
        new SequentialCommandGroup(
          new WaitUntilCommand(()->elevator1.getPosition() < 10),
          alignLeftCommand(),
        new WaitUntilCommand(()-> buttonPad.getHID().getRawButton(9) || buttonPad.getHID().getRawButton(10) || buttonPad.getHID().getRawButton(11) || buttonPad.getHID().getRawButton(12)),
        new SelectCommand<>(
          Map.ofEntries(
            Map.entry(0, new InstantCommand()),
            Map.entry(1, L1wDriveBack()),
            Map.entry(2, L2wDriveBack()),
            Map.entry(3, L3wDriveBack()),
            Map.entry(4, L4wDriveBack())
          ),
          () -> {
            if (buttonPad.getHID().getRawButton(9)) {
              return 1;
            } else if (buttonPad.getHID().getRawButton(10)) {
              return 2;
            } else if (buttonPad.getHID().getRawButton(11)) {
              return 3;
            } else if (buttonPad.getHID().getRawButton(12)) {
              return 4;
            }

            return 0;
          }
              )
            )
          );
        



controller.b().whileTrue(
  new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Right, Aligntoreef.Score.Coral, "Auto Align left"),
        Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Right, Aligntoreef.Score.Coral, "Auto Align left")
            ),
        drivePositiCommand()
    ),
    new ParallelCommandGroup(
        reefL4Command(),
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> elevator1.getPosition() > 30),
            new DriveStraight(drivetrain, 0.105).withName("Drive straight left reef"),
            new SequentialCommandGroup(
                runInTakeCommand(-12).until(() -> !coral.haveCoral()),
                runInTakeCommand(-12).withTimeout(0.4)
            ),
            new DriveStraightBack(drivetrain, 0.22).withName("Drive straight backwards")
        )
    )
  )
);
      
      
    //   controller.b().whileTrue(Aligntoreef.makeDriverController(drivetrain, elevator1, arm, Aligntoreef.Side.Right, Aligntoreef.Score.Coral, () -> {
    //   var translation = translationSupplier.get();

    //   double xMove = 0;

    //   if (translation.isPresent()) {
    //       xMove = translation.get().getX();
    //   }

    //   return xMove * 0.2;
    // }));
    // controller.povRight().whileTrue(new Aligntoreef(drivetrain, Aligntoreef.Side.Right, Aligntoreef.Score.Coral));

    arm.setDefaultCommand(drivePositiCommand());
    // vision.setDefaultCommand(new AddVisionMeasurement(drivetrain, vision).ignoringDisable(true));
    // algea.setDefaultCommand(runInTakeCommand(-8));

    // testbuttonpad
    // testButtonPad.button(1).whileTrue(elevator1.openLoopCommand(2));
    // testButtonPad.button(5).whileTrue(elevator1.openLoopCommand(-2));
    // testButtonPad.button(10).whileTrue(elevator1.pidCommand(0.5));
    // testButtonPad.button(6).whileTrue(elevator1.pidCommand(24));
    // testButtonPad.button(2).whileTrue(elevator1.pidCommand(48));

    // testButtonPad.button(3).whileTrue(arm.openLoopCommand(1));
    // testButtonPad.button(7).whileTrue(arm.openLoopCommand(-1));
    // testButtonPad.button(11).whileTrue(arm.pidCommand(20));
    // testButtonPad.button(12).whileTrue(arm.pidCommand(65));
    // testButtonPad.button(4).whileTrue(wrist.pidCommand(90));
    // testButtonPad.button(8).whileTrue(wrist.pidCommand(0));

    new Trigger(() -> !m_brakeButton.get() && DriverStation.isDisabled()).whileTrue(new StartEndCommand(
        () -> {
          LEDPattern red = LEDPattern.solid(Color.kGreen);
          red.applyTo(m_brakeLedBuffer);
          m_brakeLed.setData(m_brakeLedBuffer);

        },
        () -> {
          LEDPattern red = LEDPattern.solid(Color.kRed);
          red.applyTo(m_brakeLedBuffer);
          m_brakeLed.setData(m_brakeLedBuffer);

          arm.coastCommand();
          elevator1.coastCommand();
          wrist.coastCommand();

        }).ignoringDisable(true));

  }

  public Command getAutonomousCommand() {
    return m_autoSelector.selected();
  }


  private Command L4wDriveBack(){
    return new ParallelCommandGroup(
      reefL4Command(),
      new SequentialCommandGroup(
          new WaitUntilCommand(() -> elevator1.getPosition() > 30),
          new DriveStraight(drivetrain, 0.145).withName("Drive straight left reef"),
          new SequentialCommandGroup(
              runInTakeCommand(-12).until(() -> !coral.haveCoral()),
              runInTakeCommand(-12).withTimeout(0.4)
          ),
          new DriveStraightBack(drivetrain, 0.22).withName("Drive straight backwards")
      )
    );
  }
  private Command L3wDriveBack(){
    return new ParallelCommandGroup(
      reefL3Command(),
      new SequentialCommandGroup(
          new WaitUntilCommand(() -> elevator1.getPosition() > 30),
          new DriveStraight(drivetrain, 0.245).withName("Drive straight left reef"),
          new SequentialCommandGroup(
              runInTakeCommand(-12).until(() -> !coral.haveCoral()),
              runInTakeCommand(-12).withTimeout(0.4)
          ),
          new DriveStraightBack(drivetrain, 0.22).withName("Drive straight backwards")
      )
    );
  }
  private Command L2wDriveBack(){
    return new ParallelCommandGroup(
      reefL2Command(),
      new SequentialCommandGroup(
          new WaitUntilCommand(() -> elevator1.getPosition() > 10),
          new DriveStraight(drivetrain, 0.245).withName("Drive straight left reef"),
          new SequentialCommandGroup(
              runInTakeCommand(-12).until(() -> !coral.haveCoral()),
              runInTakeCommand(-12).withTimeout(0.4)
          ),
          new DriveStraightBack(drivetrain, 0.22).withName("Drive straight backwards")
      )
    );
  }
  private Command L1wDriveBack(){
    return new ParallelCommandGroup(
      reefL4Command(),
      new SequentialCommandGroup(
          new WaitUntilCommand(() -> elevator1.getPosition() > 30),
          new DriveStraight(drivetrain, 0.145).withName("Drive straight left reef"),
          new SequentialCommandGroup(
              runInTakeCommand(-12).until(() -> !coral.haveCoral()),
              runInTakeCommand(-12).withTimeout(0.4)
          ),
          new DriveStraightBack(drivetrain, 0.22).withName("Drive straight backwards")
      )
    );
  }



  private Command alignLeftCommand(){
    return new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Left, Aligntoreef.Score.Coral, "Auto Align left"),
        Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Left, Aligntoreef.Score.Coral, "Auto Align left")
        ),
      drivePositiCommand());
    }
    private Command alignRightCommand(){
      return new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Right, Aligntoreef.Score.Coral, "Auto Align left"),
          Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Right, Aligntoreef.Score.Coral, "Auto Align left")
          ),
        drivePositiCommand());
      }

  private Command drivePositiCommand() {
    superstructure.setRobotState(m_State.Drive);
    return new SequentialCommandGroup(
      new ConditionalCommand(
        new ParallelCommandGroup(
            arm.pidCommand(82),
            wrist.pidCommand(85)).until(()->Math.abs(arm.getPosition() - 85) < 20),
        wrist.pidCommand(85).until(() -> Math.abs(wrist.getPosition() - 85) < 10),
        () -> elevator1.getPosition() > 10),
        // wrist.pidCommand(85).until(()-> wrist.getPosition() > 80),
        // arm.pidCommand(77).until(() -> Math.abs(arm.getPosition() - 77) < 3),
        new ParallelCommandGroup(
            arm.pidCommand(77),
            wrist.pidCommand(85),
            elevator1.pidCommand(0.33).until(() -> Math.abs(elevator1.getErrorPercent()) < 2)))
        .withName("TeleOp Drive Position");
  }

  private Command autoDrivePositiCommand() {
    superstructure.setRobotState(m_State.Drive);
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new ParallelCommandGroup(
                arm.pidCommand(82),
                wrist.pidCommand(85)).until(()->Math.abs(arm.getPosition() - 77) < 3),
            wrist.pidCommand(85).until(() -> Math.abs(wrist.getPosition() - 85) < 10),
            () -> elevator1.getPosition() > 30),
        arm.pidCommand(77).until(() -> Math.abs(arm.getPosition() - 77) < 3),
        new ParallelCommandGroup(
            arm.pidCommand(77),
            wrist.pidCommand(85),
            elevator1.pidCommand(1).until(() -> Math.abs(elevator1.getErrorPercent()) < 2)

        )).withName("Auto Drive Position");
  }

  private Command pickupCommand() {
    superstructure.setRobotState(m_State.gIntake);
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.pidCommand(60),
            wrist.pidCommand(85)).until(() -> Math.abs(arm.getPosition() - 60) < 5),
        new ParallelCommandGroup(
            wrist.pidCommand(0),
            arm.coastCommand(),
            new ParallelCommandGroup(
                algea.runMotorBackwardsSpeedCommand(4),
                coral.runMotorBackwardsSpeedCommand(4)).until(coral::haveCoral)));
  }

  private Command runInTakeCommand(int voltage) {
    return new ParallelCommandGroup(
        algea.runMotorForwardsSpeedCommand(2 * voltage), coral.runMotorForwardsSpeedCommand(voltage * 2.5/4), intake.runMotorForwardsSpeedCommand(-voltage));

  }

  private Command reefL2Command() {
    superstructure.setRobotState(m_State.L2);
    return new SequentialCommandGroup(
        new WaitUntilCommand(() -> wrist.getPosition() < 90),
        arm.pidCommand(60).until(() -> Math.abs(arm.getErrorAngle()) < 3),

        new ParallelCommandGroup(
            elevator1.pidCommand(14),
            arm.pidCommand(60)).until(() -> Math.abs(elevator1.getPosition() - 14) < 3),
        new ParallelCommandGroup(
            elevator1.pidCommand(14),
            arm.pidCommand(60),
            wrist.pidCommand(131)))
        .withName("L2 Reef Command");
  }

  private Command reefL3Command() {
    superstructure.setRobotState(m_State.L3);
    return new SequentialCommandGroup(
        new WaitUntilCommand(() -> wrist.getPosition() < 90),
        arm.pidCommand(60).until(() -> Math.abs(arm.getErrorAngle()) < 3),

        new ParallelCommandGroup(
            elevator1.pidCommand(30),
            arm.pidCommand(60)).until(() -> Math.abs(elevator1.getPosition() - 30) < 1),
        new ParallelCommandGroup(
            elevator1.pidCommand(30),
            arm.pidCommand(60),
            wrist.pidCommand(125)));
  }

  private Command reefL4Command() {
    superstructure.setRobotState(m_State.L4);
    return new SequentialCommandGroup(
        new WaitUntilCommand(()-> superstructure.getRobotState() == m_State.Drive),
        arm.pidCommand(60).until(() -> Math.abs(arm.getErrorAngle()) < 3),

        new ParallelCommandGroup(
            elevator1.pidCommand(54.77),
            wrist.pidCommand(125),
            arm.pidCommand(60)).until(() -> Math.abs(elevator1.getPosition() - 54.77) < 1),
        new ParallelCommandGroup(
            elevator1.pidCommand(54.77),
            arm.pidCommand(60),
            wrist.pidCommand(125)));
  }

  private Command autoReefCommand() {
    superstructure.setRobotState(m_State.L2);
    return new SequentialCommandGroup(
        reefL2Command().until(() -> Math.abs(elevator1.getErrorPercent()) < 3)).withName("Auto Reef Command");
  }

  // private Command 
  // pickupAlgaeCommand() {
  //   superstructure.setRobotState(m_State.algaeIntake);
  //   return new SequentialCommandGroup(
  //       new ParallelCommandGroup(
  //           arm.pidCommand(60),
  //           wrist.pidCommand(60)).until(() -> Math.abs(arm.getPosition() - 60) < 5),
  //       new ParallelCommandGroup(
  //           wrist.pidCommand(20),
  //           arm.coastCommand(),
  //           new ParallelCommandGroup(algea.runMotorBackwardsSpeedCommand(8), coral.runMotorBackwardsSpeedCommand(4))).until(()-> !algea.notHaveAlgea()),
  //       new ParallelCommandGroup(
  //       drivePositiCommand(),
  //       algea.runMotorBackwardsSpeedCommand(2.6)
  //       )
        
  //           );
  // }

  private Command pickupAlgaeCommand(){
    return new SequentialCommandGroup(

              // arm.pidCommand(40).until(() -> Math.abs(arm.getPosition() - 40) < 3),
              new ParallelCommandGroup(
                  elevator1.pidCommand(8),
                  arm.pidCommand(0)).until(() -> Math.abs(elevator1.getPosition() - 8) < 2),
              new ParallelCommandGroup(
                  elevator1.pidCommand(8),
                  arm.pidCommand(0),
                  wrist.pidCommand(0))
                  );
  }

  private Command reefL1Command() {
    superstructure.setRobotState(m_State.L1);
    return new SequentialCommandGroup(
        new WaitUntilCommand(() -> wrist.getPosition() < 90),
        arm.pidCommand(78).until(() -> Math.abs(arm.getErrorAngle()) < 3),

        new ParallelCommandGroup(
            elevator1.pidCommand(5),
            arm.pidCommand(78)).until(() -> Math.abs(elevator1.getErrorPercent()) < 2),
        new ParallelCommandGroup(
            elevator1.pidCommand(5),
            arm.pidCommand(78),
            wrist.pidCommand(33)))
        .withName("L1 Reef Command");
  }

  private Command HumanPlayerIntakeCommand() {
    superstructure.setRobotState(m_State.HpIntake);
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.pidCommand(45), // arm goes down for the wrist rotate
            elevator1.pidCommand(3),
            wrist.pidCommand(173)).until(() -> Math.abs(wrist.getPosition() - 173) < 5), // wrist rotates towards the
                                                                                         // human
        // player intake
        new ParallelCommandGroup(
            elevator1.pidCommand(3),
            wrist.pidCommand(173), // hold wrist position
            arm.pidCommand(55)).until(() -> Math.abs(arm.getPosition() - 55) < 5), // arm goes up to intake from human
                                                                                   // player position
        new ParallelCommandGroup(
            elevator1.pidCommand(3),
            wrist.pidCommand(172), // hold wrist position
            arm.pidCommand(55), // hold arm position
            new SequentialCommandGroup(// runs the human player intake and then slows down after beam break sensor is
                                       // triggered
                intake.runMotorForwardsSpeedCommand(8).until(intake::haveCoral),
                new ParallelCommandGroup(
                    intake.runMotorForwardsSpeedCommand(6),
                    coral.runMotorBackwardsSpeedCommand(6)).until(coral::haveCoral)
                )))
        .withName("Human Player Intake Command");
  }

  private Command aLgaeL2Command() {
    return new SequentialCommandGroup(
        // new WaitUntilCommand(() -> wrist.getPosition() < 90),
        arm.pidCommand(40).until(() -> Math.abs(arm.getPosition() - 40) < 3),

        new ParallelCommandGroup(
            elevator1.pidCommand(31.5),
            arm.pidCommand(40)).until(() -> Math.abs(elevator1.getPosition() - 33) < 3),
        new ParallelCommandGroup(
            elevator1.pidCommand(31.5),
            arm.pidCommand(40),
            wrist.pidCommand(-23)));
  }

  private Command aLgaeL3Command() {
    superstructure.setRobotState(m_State.L4);
    return new SequentialCommandGroup(
        // new WaitUntilCommand(() -> wrist.getPosition() < 90),
        arm.pidCommand(40).until(() -> Math.abs(arm.getPosition() - 40) < 3),

        new ParallelCommandGroup(
            elevator1.pidCommand(51),
            arm.pidCommand(40)).until(() -> Math.abs(elevator1.getPosition() - 47) < 3),
        new ParallelCommandGroup(
            elevator1.pidCommand(51),
            arm.pidCommand(40),
            wrist.pidCommand(-23)));

  }


  private Command aLgaeL3CommandV2(){
    return new SequentialCommandGroup(
      elevator1.pidCommand(51).until(()-> Math.abs(elevator1.getPosition() - 51) < 3),
      new ParallelCommandGroup(
        elevator1.pidCommand(51),
        arm.pidCommand(40),
        wrist.pidCommand(-23)
      )


    );
  }

  private Command processorCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(

            arm.pidCommand(72),
            elevator1.pidCommand(3)
            // wrist.pidCommand(4)).until(() -> Math.abs(arm.getPosition() - 72) < 5),
        ).until(() -> Math.abs(arm.getPosition() - 72) < 5),
        new ParallelCommandGroup(
            elevator1.pidCommand(3),
            wrist.pidCommand(4),
            arm.pidCommand(72)
        ));
  }

  private Command dropIntake() {
    return new ParallelCommandGroup(
        intakeServoRight.engageServo(),
        intakeServoLeft.engageServo());
  }
  private Command testUndropIntake() {
    return new ParallelCommandGroup(
      intakeServoRight.setPositionCommand(0.24),
      intakeServoLeft.setPositionCommand(0.75)
    ); 
  }
  private Command dropServoCommand ()
  {
    return new ParallelCommandGroup(
      intakeServoRight.setPositionCommand(0.0),
      intakeServoLeft.setPositionCommand(0.9)
    );
  }


  private Command netCommand() {
    superstructure.setRobotState(m_State.L4);
    return new SequentialCommandGroup(
        new WaitUntilCommand(()-> superstructure.getRobotState() == m_State.Drive),
        arm.pidCommand(85).until(() -> Math.abs(arm.getErrorAngle()) < 3),

        new ParallelCommandGroup(
            elevator1.pidCommand(60),
            arm.pidCommand(85)).until(() -> Math.abs(elevator1.getPosition() - 60) < 1),
        new ParallelCommandGroup(
            elevator1.pidCommand(60),
            arm.pidCommand(85),
            wrist.pidCommand(125)));
  }

  private Command setVisionPose(double xMeters, double yMeters) {
    return new InstantCommand(()->{
      drivetrain.resetPose(new Pose2d(new Translation2d(xMeters, yMeters), drivetrain.getState().Pose.getRotation()));
    });

  }

  // public Command AddVisionMeasurement() {
  //   return new AddVisionMeasurement(drivetrain, vision)
  //       .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  // }
  private Command autoClimbCommand() {
    return new SequentialCommandGroup(
        m_Climber.extendCommand().until(()-> m_Climber.getPosition() < 0.22),
        testUndropIntake().withTimeout(1.5),
        m_Climber.extendCommand());
  }

  private Command visionLeftL4(){
    return new SequentialCommandGroup(
      Aligntoreef.makeAuto(drivetrain, elevator1, arm, Aligntoreef.Side.Left, Aligntoreef.Score.Coral, "Vision Left")
    );
  }
}

