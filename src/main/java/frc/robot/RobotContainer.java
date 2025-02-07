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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(4).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                 // angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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

    configureBindings();
    m_autoSelector.initialize();

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Drive", drivetrain);
  }

  private void configureBindings() {

    //
    // Drive Train
    //

    final var xFilter = new SlewRateLimiter(5);
    final var yFilter = new SlewRateLimiter(5);
    final var rotateFilter = new SlewRateLimiter(5);

    BooleanSupplier slowModeSupplier = () -> joystick.getHID().getXButton();

    DoubleSupplier rotationSupplier = () -> {
      double leftTrigger = joystick.getHID().getLeftTriggerAxis();
      double rightTrigger = joystick.getHID().getRightTriggerAxis();

      double rotate = 0.0;
      if (leftTrigger < rightTrigger) {
        rotate = -rightTrigger;
      } else {
        rotate = leftTrigger;
      }

      return rotateFilter.calculate(rotate) * MaxAngularRate;
    };

    Supplier<Optional<Translation2d>> translationSupplier = () -> {
      // Read gamepad joystick state, and apply slew rate limiters

      // X Move Velocity - Forward
      double xMove = MathUtil.applyDeadband(xFilter.calculate(-joystick.getHID().getLeftY()), .05);
      
      // Y Move Velocity - Strafe
      double yMove = MathUtil.applyDeadband(yFilter.calculate(-joystick.getHID().getLeftX()), .05);

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
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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

    joystick.a().whileTrue(drivetrain.applyRequest(() -> {
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
    joystick.rightStick().toggleOnTrue(drivetrain.applyRequest(() -> brake));

    // Reset the field-centric heading on start press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return m_autoSelector.selected();
  }

}
