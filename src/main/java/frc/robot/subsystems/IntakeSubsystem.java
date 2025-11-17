// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.configs.IntakeConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // roller and feeder motors are NEO 550s (brushless)
  private SparkMax mRoller;
  private SparkMax mFeeder;
  // this array holds all of our limit switches in the intake system
  private DigitalInput[] mLimitSwitches;

  // constructor
  public IntakeSubsystem() {
    mRoller = new SparkMax(IntakeConstants.ROLLER_CAN_ID, MotorType.kBrushless);
    mFeeder = new SparkMax(IntakeConstants.FEEDER_CAN_ID, MotorType.kBrushless);

    // apply configurations to SparkMax motor controllers 
    mRoller.configure(IntakeConfig.rollerConfig,
                      ResetMode.kResetSafeParameters,
                      PersistMode.kPersistParameters);
    mFeeder.configure(IntakeConfig.feederConfig,
                      ResetMode.kResetSafeParameters,
                      PersistMode.kPersistParameters);
    // configure limit switch array
    mLimitSwitches = new DigitalInput[IntakeConstants.MAX_LIMIT_SWITCHES];
    for (int i=0; i < mLimitSwitches.length; i++)
    {
        mLimitSwitches[i] = new DigitalInput(i);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
