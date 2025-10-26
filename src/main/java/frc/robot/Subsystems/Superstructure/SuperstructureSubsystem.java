package frc.robot.Subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Superstructure.EndEffector.EndEffectorSubsystem;

public class SuperstructureSubsystem extends SubsystemBase{
    private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
    private final EndEffectorSubsystem mEndEffectorSubsystem = new EndEffectorSubsystem();

    private final ElevatorSim elevatorSim = mElevatorSubsystem.getSim();
    private final SingleJointedArmSim armSim = mEndEffectorSubsystem.getSim();

    private final Mechanism2d mech2d = new Mechanism2d(50,160,new Color8Bit(199, 235, 209));
    private final MechanismRoot2d mech2droot = mech2d.getRoot("Elevator root", 25, 0);
    private final MechanismLigament2d elevator = mech2droot.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 
        90,5,new Color8Bit(228, 103, 245)));

    private final MechanismLigament2d arm = elevator.append( 
        new MechanismLigament2d(
            "Arm",
            20,
            Units.radiansToDegrees(armSim.getAngleRads()),
            3,
            new Color8Bit(245, 103, 176)));

    public SuperstructureSubsystem(){
        SmartDashboard.putData("Mechanism",mech2d);
    }

    @Override
    public void periodic(){
        elevator.setLength(mElevatorSubsystem.setElevatorSimLength());
        arm.setAngle(mEndEffectorSubsystem.setArmSimAngle()-90);
    }

    @Override
    public void simulationPeriodic(){

    }

    //Elevator

    public Command reachGoalCmd(double goal){
        return mElevatorSubsystem.reachGoalCmd(goal);
    }

    public Command holdPositionCmd(){
        return mElevatorSubsystem.holdPositionCmd();
    }

    public Command stopElevatorCmd(){
        return mElevatorSubsystem.stopCmd();
    }

    //End effector
    public Command reachSetpointCmd(double setpoint){
        return mEndEffectorSubsystem.reachSetpointCmd(setpoint);
    }

    public Command holdAngleCmd(){
        return mEndEffectorSubsystem.holdAngleCmd();
    }

    public Command stopArm(){
        return mEndEffectorSubsystem.stopArmCmd();
    }

    public Command testSpeedCmd(double var){
        return mEndEffectorSubsystem.testSpeedCmd(var);
    }
}