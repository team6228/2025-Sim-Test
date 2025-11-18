package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.DriveSubsystem;

import java.util.function.Supplier;

public class CartesianDriveCmd extends Command{
    private final DriveSubsystem mDriveSubsystem;
    private final Supplier<Double> mSpeedXFunc,mSpeedYFunc,mRotationZFunc;


    public CartesianDriveCmd(DriveSubsystem driveSubsystem,Supplier<Double> speedXFunc,
            Supplier<Double> speedYFunc,Supplier<Double> rotationZFunc){
        this.mDriveSubsystem = driveSubsystem;
        this.mSpeedXFunc = speedXFunc;
        this.mSpeedYFunc = speedYFunc;
        this.mRotationZFunc = rotationZFunc;

        addRequirements(driveSubsystem);
    }

    @Override 
    public void initialize(){

    }

    @Override
    public void execute(){
        Double speedX = mSpeedXFunc.get();
        Double speedY = mSpeedYFunc.get();
        Double rotationZ = mRotationZFunc.get();

        mDriveSubsystem.cartesianDrive(speedX,speedY,rotationZ);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
