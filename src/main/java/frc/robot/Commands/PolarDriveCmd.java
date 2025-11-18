package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.DriveSubsystem;

public class PolarDriveCmd extends Command{
    private final DriveSubsystem mDriveSubsystem;
    //[TODO] add needed suppliers

    public PolarDriveCmd(DriveSubsystem driveSubsystem){
        this.mDriveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        mDriveSubsystem.polarDrive();
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
