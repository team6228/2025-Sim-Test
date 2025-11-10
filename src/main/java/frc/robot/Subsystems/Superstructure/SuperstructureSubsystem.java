package frc.robot.Subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final MechanismRoot2d mech2droot = mech2d.getRoot("Elevator root", 25, 0.0925);
    private final MechanismLigament2d elevator = mech2droot.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 
        90,5,new Color8Bit(228, 103, 245)));

    private final MechanismLigament2d arm = elevator.append( 
        new MechanismLigament2d(
            "Arm",
            .35,
            Units.radiansToDegrees(armSim.getAngleRads()),
            3,
            new Color8Bit(245, 103, 176)));

    //could add to constants
    private double stageOffset = 0.03; //3cm
    private double elevMechOffset = 0.0925;
    private double armXOffset = 0.3575;
    private double armZOffset = 0.18;
    
    private final Pose3d zeroedComponantPoses = new Pose3d(0,0,0,new Rotation3d());
    //0.55,0.3335,-0.0725
    private Pose3d finalStage2Pose = new Pose3d(0,0,
        0+stageOffset*1+elevMechOffset + elevator.getLength(),new Rotation3d());
    private Pose3d finalStage3Pose = new Pose3d(0,0,
        0+stageOffset*2+elevMechOffset + elevator.getLength(),new Rotation3d());
    private Pose3d finalStage4Pose = new Pose3d(0,0,
        0+stageOffset*3-elevMechOffset + elevator.getLength(),new Rotation3d());
    //might change the rotation
    private Pose3d finalArmPose = new Pose3d(armXOffset,0,
        0+stageOffset*3+elevMechOffset + elevator.getLength()*3 + armZOffset,
        new Rotation3d(0,Units.degreesToRadians(arm.getAngle()),0));

    private StructPublisher<Pose3d> zeroedPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("zeroedComponantPoses", Pose3d.struct).publish();
    private StructPublisher<Pose3d> stage2Publisher = NetworkTableInstance.getDefault()
        .getStructTopic("finalStage2Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> stage3Publisher = NetworkTableInstance.getDefault()
        .getStructTopic("finalStage3Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> stage4Publisher = NetworkTableInstance.getDefault()
        .getStructTopic("finalStage4Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> armPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("finalArmPose", Pose3d.struct).publish();

    public SuperstructureSubsystem(){
        SmartDashboard.putData("Mechanism",mech2d);
        zeroedPublisher.set(zeroedComponantPoses);
        stage2Publisher.set(finalStage2Pose);
        stage3Publisher.set(finalStage3Pose);
        stage4Publisher.set(finalStage4Pose);
        armPublisher.set(finalArmPose);
    }

    @Override
    public void periodic(){
        elevator.setLength(mElevatorSubsystem.setElevatorSimLength());
        arm.setAngle(-mEndEffectorSubsystem.setArmSimAngle()-70);

        /* 
        finalStage2Pose = new Pose3d(0.1425,0,
            0+stageOffset*1+elevMechOffset + elevator.getLength(),new Rotation3d());
        finalStage3Pose = new Pose3d(0.1425,0,
            0+stageOffset*2+elevMechOffset + elevator.getLength(),new Rotation3d());
        finalStage4Pose = new Pose3d(0,0,
            0+stageOffset*3+elevMechOffset + elevator.getLength(),new Rotation3d());
        */
        finalStage2Pose = new Pose3d(0.1425,0,
            0+stageOffset*1+elevMechOffset + elevator.getLength()*1,new Rotation3d());
        finalStage3Pose = new Pose3d(0.1425,0,
            0+stageOffset*2+elevMechOffset + elevator.getLength()*2,new Rotation3d());
        finalStage4Pose = new Pose3d(0.1425,0,
            0+stageOffset*3+elevMechOffset + elevator.getLength()*3,new Rotation3d());

        //might change the rotation
        finalArmPose = new Pose3d(armXOffset,0,
            0+stageOffset*3+elevMechOffset + elevator.getLength()*3 + armZOffset,
            new Rotation3d(0,Units.degreesToRadians(arm.getAngle()),0));
    
        zeroedPublisher.set(zeroedComponantPoses);
        stage2Publisher.set(finalStage2Pose);
        stage3Publisher.set(finalStage3Pose);
        stage4Publisher.set(finalStage4Pose);
        armPublisher.set(finalArmPose);
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