// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity (length, complexity).
 */
public final class Constants {
  public static final class DriveConstants{ 
    //allowed max speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; //rad/s 

    //Chassis config - width, depth, CAN IDS and angular offset values in Desgndoc.md
    public static final double kFrontLeftWidth = Units.inchesToMeters(5.245126);
    public static final double kFrontRightWidth = Units.inchesToMeters(5.245126);
    public static final double kBackLeftWidth = Units.inchesToMeters(6.250000);
    public static final double kBackRightWidth = Units.inchesToMeters(6.250000);
    // horizontal width/distance from robot center to each wheel 
    public static final double kFrontLeftDepth = Units.inchesToMeters(4.286576);
    public static final double kFrontRightDepth = Units.inchesToMeters(4.286576);
    public static final double kBackLeftDepth = Units.inchesToMeters(6.250000);
    public static final double kBackRightDepth = Units.inchesToMeters(6.250000);
    //depth/frontback distance from robot center to each wheel 

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kFrontLeftDepth, kFrontLeftWidth),
        new Translation2d(kFrontRightDepth, -kFrontRightWidth),
        new Translation2d(-kBackLeftDepth, kBackLeftWidth),
        new Translation2d(-kBackRightDepth, -kBackRightWidth));
      
    //angular offsets of module relative to chassis (rad)
    public static final double kFrontLeftChassisAngularOffset = Math.PI/4;
    public static final double kFrontRightChassisAngularOffset = -Math.PI/4;
    public static final double kBackLeftChassisAngularOffset = -Math.PI/2;
    public static final double kBackRightChassisAngularOffset = 0;
    
    //SPARK MAX CAN IDs 
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontLeftTurningCanId = 2;

    
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kFrontRightTurningCanId = 8;
    
    public static final int kBackLeftDrivingCanId = 3;
    public static final int kBackLeftTurningCanId = 4;
   
    public static final int kBackRightDrivingCanId = 5;
    public static final int kBackRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants{
    // The MAXSwerve module 3 pinion ggears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (more teeth = faster)
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05; 
  }

  //TODO: go over and update auto constants according to necessary auto commands
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

}
