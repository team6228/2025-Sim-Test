package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Drive.DriveSubsystem;

public class ArcadeDriveCmd extends Command{
    private final DriveSubsystem mDriveSubsystem;
    private final Supplier<Double> mFwdFunction,mRotFunction;

    public ArcadeDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> fwdFunction, Supplier<Double> rotFunction) {
        this.mDriveSubsystem = driveSubsystem;
        this.mFwdFunction = fwdFunction;
        this.mRotFunction = rotFunction;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        Double fwd = mFwdFunction.get();
        double rot = mRotFunction.get();

        mDriveSubsystem.arcadeDrive(fwd,rot);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
