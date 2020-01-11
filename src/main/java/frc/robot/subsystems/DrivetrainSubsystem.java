/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.model.SwerveModule;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DrivetrainSubsystem.
   */

  Translation2d frontLeftLocation = new Translation2d(Constants.WHEELBASE / 2, Constants.TRACKWIDTH / 2);
  Translation2d frontRightLocation = new Translation2d(Constants.WHEELBASE / 2, -Constants.TRACKWIDTH / 2);
  Translation2d rearLeftLocation = new Translation2d(-Constants.WHEELBASE / 2, Constants.TRACKWIDTH / 2);
  Translation2d rearRightLocation = new Translation2d(-Constants.WHEELBASE / 2, -Constants.TRACKWIDTH / 2);

  SwerveModule frontLeftModule = new SwerveModule(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER, Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_OFFSET);
  SwerveModule frontRightModule = new SwerveModule(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER, Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_OFFSET);
  SwerveModule rearLeftModule = new SwerveModule(Constants.DRIVETRAIN_REAR_LEFT_DRIVE_MOTOR, Constants.DRIVETRAIN_REAR_LEFT_ANGLE_MOTOR, Constants.DRIVETRAIN_REAR_LEFT_ANGLE_ENCODER, Constants.DRIVETRAIN_REAR_LEFT_ANGLE_OFFSET);
  SwerveModule rearRightModule = new SwerveModule(Constants.DRIVETRAIN_REAR_RIGHT_DRIVE_MOTOR, Constants.DRIVETRAIN_REAR_RIGHT_ANGLE_MOTOR, Constants.DRIVETRAIN_REAR_RIGHT_ANGLE_ENCODER, Constants.DRIVETRAIN_REAR_RIGHT_ANGLE_OFFSET);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation
  );

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle());
 
  AHRS navX;

  public DrivetrainSubsystem() {
    navX = new AHRS(SPI.Port.kMXP);
    if (navX != null) {
      System.out.println("resetting navX");
      navX.reset();
    }
     
  }
  
  public Rotation2d getAngle() {
    if (navX != null) {
      //System.out.println(navX.getAngle());
      SmartDashboard.putNumber("navX Angle: ", navX.getAngle());
      return Rotation2d.fromDegrees(navX.getAngle());
    } else {
      System.out.println("navX is null");
      return new Rotation2d(0,0);
    }

  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (navX == null) {
      System.out.println("drive: navX is null");
      return;
    }
    //System.out.println("drive xSpeed:" + xSpeed + "\tySpeed:" + ySpeed);
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.SWERVE_MAX_VELOCITY);

    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  public void dashboardAngleEncoders() {
      SmartDashboard.putNumber("Front Left", frontLeftModule.getAdjustedAngleEncoder());
      SmartDashboard.putNumber("Front Right", frontRightModule.getAdjustedAngleEncoder());
      SmartDashboard.putNumber("Rear Left", rearLeftModule.getAdjustedAngleEncoder());
      SmartDashboard.putNumber("Rear Right", rearRightModule.getAdjustedAngleEncoder());
    }
  public void updateOdometry() {
    odometry.update(
      getAngle(),
      frontLeftModule.getState(),
      frontRightModule.getState(),
      rearLeftModule.getState(),
      rearRightModule.getState()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }
}
