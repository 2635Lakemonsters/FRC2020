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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.model.SwerveModule;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DrivetrainSubsystem.
   */

  Translation2d frontLeftLocation = new Translation2d(0,0);
  Translation2d frontRightLocation = new Translation2d(0,0);
  Translation2d rearLeftLocation = new Translation2d(0,0);
  Translation2d rearRightLocation = new Translation2d(0,0);

  SwerveModule frontLeftModule = new SwerveModule(0,0,0,0);
  SwerveModule frontRightModule = new SwerveModule(0,0,0,0);
  SwerveModule rearLeftModule = new SwerveModule(0,0,0,0);
  SwerveModule rearRightModule = new SwerveModule(0,0,0,0);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation
  );

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle());
 
  AHRS navX = new AHRS(SPI.Port.kMXP);

  public DrivetrainSubsystem() {
    navX.reset();
  }
  
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(navX.getAngle());
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.SWERVE_MAX_VELOCITY);

    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
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
  }
}
