// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.Commands.CartesianDriveCmd;
import frc.robot.Commands.PolarDriveCmd;

import frc.robot.Constants.OperatorConstants;

import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Superstructure.SuperstructureSubsystem;

public class RobotContainer {
  private final DriveSubsystem mDrive = new DriveSubsystem();
  private final SuperstructureSubsystem mSuperstructure = new SuperstructureSubsystem();

  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.kJoystickPort);

  public RobotContainer() {
    configureBindings();

    //Ps4
    //mDrive.setDefaultCommand(new ArcadeDriveCmd(mDrive,() -> joystick.getRawAxis(1),() ->joystick.getRawAxis(2)));
    //Keyboard
    //mDrive.setDefaultCommand(new ArcadeDriveCmd(mDrive,() -> joystick.getRawAxis(1),() ->joystick.getRawAxis(0)));
  }

  private void configureBindings() {
    //[TODO] get button from constants
    //Keyboard
    
    joystick.button(1).toggleOnTrue(mSuperstructure.reachGoalCmd(0.00));
    joystick.button(2).toggleOnTrue(mSuperstructure.reachGoalCmd(1.20));

    joystick.button(3).toggleOnTrue(mSuperstructure.reachSetpointCmd(-25.00));
    joystick.button(4).toggleOnTrue(mSuperstructure.reachSetpointCmd(0.00));
    
    //Ps4
    /* 
    joystick.button(1).toggleOnTrue(mSuperstructure.reachGoalCmd(0.00));
    joystick.button(2).toggleOnTrue(mSuperstructure.reachGoalCmd(0.50));
    joystick.button(3).toggleOnTrue(mSuperstructure.reachGoalCmd(1.25));
    joystick.button(4).toggleOnTrue(mSuperstructure.reachGoalCmd(1.80));

    joystick.pov(0).toggleOnTrue(mSuperstructure.reachSetpointCmd(-45.00));
    joystick.pov(90).toggleOnTrue(mSuperstructure.reachSetpointCmd(0.00));
    joystick.pov(180).toggleOnTrue(mSuperstructure.reachSetpointCmd(45.00));
    joystick.pov(270).toggleOnTrue(mSuperstructure.reachSetpointCmd(90.00));
    */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
