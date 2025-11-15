// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  //create 4 MAXSwerveModules 
  
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);
  
  private final MAXSwerveModule frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);
  
  private final MAXSwerveModule backLeft = new MAXSwerveModule(
    DriveConstants.kBackLeftDrivingCanId,
    DriveConstants.kBackLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule backRight = new MAXSwerveModule(
    DriveConstants.kBackLeftDrivingCanId,
    DriveConstants.kBackLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

  //gyro sensor/IMU (usb input type to roborio)
  private final AHRS gyro = new AHRS(NavXComType.kUSB1); 

  //odometry class for tracking robot pose 
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(-gyro.getAngle()), //inversion as NavX is CCW+
    new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
  });

  
  public DriveSubsystem() {
    //usage reporting for MAXSwerve template 
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic(){
  //updates odometry in periodic block 
    odometry.update(
        Rotation2d.fromDegrees(-gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(-gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    //convert joystick input (-1, 1) to m/s for drivetrain 
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond; 
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    //convert chassis speed to swerve module states (motor output); field relative or robot relative 
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, 
          rotDelivered, Rotation2d.fromDegrees(-gyro.getAngle()))
        
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
}
