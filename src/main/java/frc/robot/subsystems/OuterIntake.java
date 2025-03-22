package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.compound.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.traits.*;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;
import java.util.HashMap;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class OuterIntake  extends SubsystemBase {
    // private final CANdi algaeIntake;
    private TalonFXSConfiguration configs_FXS;
    private TalonFXS algaeTalonFXS1;
    private final CANdi caNdi;
    public CANdi getCANDI() {return caNdi;}

    public OuterIntake (int canDIid, int motorid, boolean isInverted) {
        algaeTalonFXS1 = new TalonFXS(motorid, "rio");
        
        configs_FXS = new TalonFXSConfiguration();
        configs_FXS.Slot0.kP = 1;
        configs_FXS.Slot0.kI = 0;
        configs_FXS.Slot0.kD = 10;
        configs_FXS.Slot0.kV = 2;
        configs_FXS.CurrentLimits.StatorCurrentLimitEnable = true;
        configs_FXS.CurrentLimits.StatorCurrentLimit = 40;
        configs_FXS.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        configs_FXS.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        caNdi = new CANdi(canDIid, "rio");

        CANdiConfiguration configs_CANdi = new CANdiConfiguration();
        
        // Beam break
        configs_CANdi.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;

        // Lamprey Encoder
        configs_CANdi.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
        configs_CANdi.PWM2.AbsoluteSensorDiscontinuityPoint = 0.5;
        configs_CANdi.PWM2.SensorDirection = false;
        configs_CANdi.PWM2.AbsoluteSensorOffset =0;
        

        caNdi.getConfigurator().apply(configs_CANdi);
        caNdi.getS1Closed().setUpdateFrequency(100);
        caNdi.getS2Closed().setUpdateFrequency(100);

        // algaeIntake = new CANdi(motorid, "Canivore");

        // CANdiConfiguration configs_CANdi = new CANdiConfiguration();
        
        // configs_CANdi.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
        // configs_CANdi.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        // configs_CANdi.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        // configs_CANdi.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;
        // // algaeIntake.getConfigurator().apply(configs_CANdi);
        // // algaeIntake.getS1Closed().setUpdateFrequency(100);
        // // algaeIntake.getS2Closed().setUpdateFrequency(100);

        algaeTalonFXS1.getConfigurator().apply(configs_FXS);

        
    }
    // public boolean getS1Closed() {
    //     return algaeIntake.getS1Closed().getValue(); 
    // }
    // public boolean getS2Closed() {
    //     return algaeIntake.getS2Closed().getValue();
    // }
    
    public void stop() {
        algaeTalonFXS1.stopMotor();
    }
    public double getPos() {
        return algaeTalonFXS1.getPosition().getValueAsDouble();
    }
    public double getVelo() {
        return algaeTalonFXS1.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("S1_Closed", getS1Closed());
        // SmartDashboard.putBoolean("S2_Closed", getS2Closed());
        SmartDashboard.putNumber("OuterIntake position", getPos());
        SmartDashboard.putNumber("OuterIntake velocity", getVelo());
    }
    public Command runMotorForwardsCommand() {
	    return Commands.runEnd(
            
	        () -> algaeTalonFXS1.setVoltage(1), this::stop, this);
        

    }
    public Command runMotorForwardsSpeedCommand(double motorVoltage) {
	    return Commands.runEnd(
            
	        () -> algaeTalonFXS1.setVoltage(motorVoltage), this::stop, this);
        

    }
    public Command runMotorBackwardsSpeedCommand(double motorVoltage) {
	    return Commands.runEnd(
            
	        () -> algaeTalonFXS1.setVoltage(-motorVoltage), this::stop, this);
        

    }
     public Command slowMotorAtSensorCommand() {
        return new SequentialCommandGroup(
            // runMotorForwardsSpeedCommand(3).until(()-> getS1Closed()),
            runMotorForwardsSpeedCommand(1)
            //runMotorBackwardsSpeedCommand(3)

            //setPositCommand()


        );
    }
    public Command stopMotorCommand() {
        return Commands.runEnd(() -> algaeTalonFXS1.setVoltage(0), this::stop, this);
    }

    public boolean notHaveAlgea() {
        return !caNdi.getS1Closed().getValue(); 
    }

    
}
