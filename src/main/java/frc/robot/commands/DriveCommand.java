/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
  /**
   * Creates a new DriveCommand.
   */
  DrivetrainSubsystem drivetrainSubsystem;
  Joystick leftStick;
  Joystick rightStick;
  public DriveCommand(Joystick leftStick, Joystick rightStick, DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    this.leftStick = leftStick;
    this.rightStick = rightStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left1 = -leftStick.getRawAxis(1);
    double left0 = -leftStick.getRawAxis(0);
    double right0 = rightStick.getRawAxis(0);

    if(Math.abs(left1) < 0.05){
      left1 = 0;
    }
    if(Math.abs(left0) < 0.05){
      left0 = 0;
    }
    if(Math.abs(right0) < 0.05){
      right0 = 0;
    }

    drivetrainSubsystem.drive(left1, left0, right0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
