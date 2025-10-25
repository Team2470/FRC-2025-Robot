package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Superstructure.SuperstructurePosition.*;

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
    private TargetAction selectedTargetAction = TargetAction.HOME;
    private TargetAction currentAction = TargetAction.HOME;
    private TargetAction previousAction;
    private boolean isChangingState;

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
        TargetAction goalTargetAction = getCurrentAction();
        if (goalTargetAction != previousAction) {
            m_arm.setPosition(goalTargetAction.getArmAngle());
        } 
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

    public boolean armAtPosition(TargetAction goalPosition) {
        return m_arm.getPosition() == goalPosition.getArmAngle();
    }

    public boolean elevatorAtPosition(TargetAction goalPosition) {
        return m_elevator.getPosition() == goalPosition.getElevatorPosition();
    }

    public boolean wristAtPosition(TargetAction goalPosition) {
        return m_wrist.getPosition() == goalPosition.getWristAngle();
    }

    public boolean atPosition(TargetAction goalPosition) {
        return elevatorAtPosition(goalPosition)
                && armAtPosition(goalPosition)
                && wristAtPosition(goalPosition);
    }

    public void setCurrentAction(TargetAction target) {
        currentAction = target;
    }

    public Command confirm() {
        return new InstantCommand(() -> confirmSelectedAction());
    }

    public void confirmSelectedAction() {
        currentAction = selectedTargetAction;
        revealCombination();
    }

    public TargetAction getSelectedTargetAction() {
        return selectedTargetAction;
    }

    public TargetAction getCurrentAction() {
        return currentAction;
    }

    public void revealCombination() {
        System.out.println("Goal : " + getSelectedTargetAction().toString());
    }
}
