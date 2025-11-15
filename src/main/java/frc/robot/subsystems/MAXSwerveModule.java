// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class MAXSwerveModule extends SubsystemBase{ //TO DO: PROBABLY NEED TO EXTEND SUBSYSTEM BASE 
    
    private final SparkMax drivingSpark; 
    private final SparkMax turningSpark; 

    private final RelativeEncoder drivingEncoder; 
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController; 
    private final SparkClosedLoopController turningClosedLoopController; 

    private double m_chassisAngularOffset = 0; 
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d()); 

   
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. ONLY FOR REVMAXSWERVE MODULE 
     * built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */

    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
      drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless); 
      turningSpark = new SparkMax(turningCANId, MotorType.kBrushless); 

      drivingEncoder = drivingSpark.getEncoder();
      turningEncoder = turningSpark.getAbsoluteEncoder();

      drivingClosedLoopController = drivingSpark.getClosedLoopController();
      turningClosedLoopController = turningSpark.getClosedLoopController(); 

      // Apply the respective configurations to the SPARKS. Reset parameters before
      //applying the configuration to bring the SPARK to a known good state. Persist
      // the settings to avoid losing them on a power cycle.

      drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters); 
      turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

      m_chassisAngularOffset = chassisAngularOffset; 
      
      m_desiredState.angle = new Rotation2d(turningEncoder.getPosition()); 
      drivingEncoder.setPosition(0); 
   }

   /**
   * @return The current state of the module = velocity (rad/s) + rotational angle (rad)
   */
   public SwerveModuleState getState() {
      return new SwerveModuleState(drivingEncoder.getVelocity(), 
        new Rotation2d(turningEncoder.getPosition() - m_chassisAngularOffset)); 
      //encoder position relative to chassis corrected with offset of module 
   }
  
   /**
   * @return The current position of the module = position(m) + rotational angle (rad) 
   */
   public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(drivingEncoder.getPosition(),
      new Rotation2d(turningEncoder.getPosition()- m_chassisAngularOffset));
   }

   public void setDesiredState(SwerveModuleState desiredState){
      SwerveModuleState correctedDesiredState = new SwerveModuleState(); 
      correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond; //velocity from drive method
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)); 
      //applies angular offset to desired state

      // Optimize the reference state to avoid spinning further than 90 degrees (takes shortest rotation)
      correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));
      
      //set speed and rotational setpoint for driving and turning Sparkmaxs 
      drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity); 
      turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition); 

      m_desiredState = desiredState;
   }

   //zero swervemodule drive encoders 
   public void resetEncoders(){
      drivingEncoder.setPosition(0);
   }


}
