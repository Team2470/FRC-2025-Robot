package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Superstructure extends SubsystemBase{
    

    public enum m_State {
        Drive, HpIntake, gIntake, algaeIntake, L1, L2, L3, L4
    }

    public m_State currentState;

    

    private double elevatorLength;
    private double armLength = 16;
    private double wristLength = 12.9;
    private double elevatorAngle = 0.174533;
    private double armAngle;
    private double wristAngle;
    private double heightLimit;
    private double distanceLimit;
    private double elevatorX;
    private double elevatorY;
    private double armX;
    private double armY;
    private double wristX;
    private double wristY;


    private final Arm m_arm;
    private final Elevator m_elevator;
    private final Wrist m_wrist;

    public Superstructure(Arm arm, Elevator elevator, Wrist wrist) {
        currentState = m_State.Drive;
        
        m_arm = arm;
        m_elevator = elevator;
        m_wrist = wrist;

        
        

    }

    public void setRobotState(m_State state) {
        currentState = state;
      }

    public m_State getRobotState() {
        return currentState;
    }

    @Override
    public void periodic() {



        armAngle = m_arm.getPosition();
        wristAngle = m_wrist.getPosition();
        elevatorLength = m_elevator.getPosition();
        updatePositions();


        SmartDashboard.putString("Super Structure State", currentState.toString());
        SmartDashboard.putNumber("Elevator Height", elevatorLength);
        SmartDashboard.putNumber("Arm Angle", armAngle);
        SmartDashboard.putNumber("Wrist Angle", wristAngle);
        SmartDashboard.putNumber("Wrist X", wristX);
        SmartDashboard.putString("RobotState", currentState.toString());
    }

    public void updateElevatorPosition(){
        elevatorX = elevatorLength * Math.sin(elevatorAngle);
        elevatorY = elevatorLength * Math.cos(elevatorAngle);
    }

    public void updateArmPosition(){

            updateElevatorPosition();
            armX = elevatorX - armLength * Math.sin(armAngle-elevatorAngle);
            armY = elevatorY - armLength * Math.cos(armAngle-elevatorAngle);

    }

    public void updateWristPosition(){

        updateArmPosition();
        wristX = armX + wristLength * Math.sin(wristAngle - armAngle + elevatorAngle);
        wristY = armY + wristLength * Math.cos(wristAngle - armAngle + elevatorAngle);

    }

    public void updatePositions(){

        elevatorX = elevatorLength * Math.sin(elevatorAngle);
        elevatorY = elevatorLength * Math.cos(elevatorAngle);        
        armX = elevatorX - armLength * Math.sin(armAngle-elevatorAngle);
        armY = elevatorY - armLength * Math.cos(armAngle-elevatorAngle);
        wristX = armX + wristLength * Math.sin(wristAngle - armAngle + elevatorAngle);
        wristY = armY + wristLength * Math.cos(wristAngle - armAngle + elevatorAngle);

    }

    public double getElevatorX(){

        return elevatorX;

    }

    public double getElevatorY(){

        return elevatorY;

    }

    public double getArmX(){

        return armX;

    }

    public double getArmY(){

        return armY;

    }

    public double getWristX(){

        return wristX;

    }

    public double getWristY(){

        return wristY;

    }

}
