package frc.robot.Subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Superstructure.Elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperstructureSubsystem extends SubsystemBase(){
    private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();

    public SuperstructureSubsystem(){

    }


}