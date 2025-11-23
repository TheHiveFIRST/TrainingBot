// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.configs.IntakeConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // roller and feeder motors are NEO 550s (brushless)
  private SparkMax mRoller;
  private SparkMax mFeeder;
  // this can be used for telemetry
  private int mNumArtifacts = 0;
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

  /* 
   * this function will power the roller and feeder into our robot
   * arg:
   * speed: the speed of the feeder and the roller
   */
  private void runIntakeSystem(double speed)
  {
    mFeeder.set(speed);
    mRoller.set(speed);
  }

  /*
   * this function checks if any artifacts are in the robot
   * returns:
   * True if any of the limit switches has a ball
   * False if there is no ball in the system 
   */
  public boolean hasArtifacts()
  {
    boolean result = false;
    for (int i=0; i < mLimitSwitches.length; i++)
    {
        if (mLimitSwitches[i].get())
        {
            result = true;
        }
    }
    return result;
  }

  /*
   * this function get the number of artifacts within the robot
   * returns:
   * number of artifacts in the robot
   */
  public int getNumArtifacts()
  {
    int result = 0;
    for (int i=0; i < mLimitSwitches.length; i++)
    {
        if (mLimitSwitches[i].get())
        {
            result++;
        }
    }

    return result;
  }

  /*
   * This command will repeatedly run our intake. This can be ued for intaking or outtaking
   * args:
   * speed: the speed to run our intake system at
   */
  public Command intakeArtifacts(double speed) {
    // Inline construction of command goes here.
    return run(
        () -> {
            if (getNumArtifacts() != 3)
            {
                runIntakeSystem(speed);
            }
            else
            {
                runIntakeSystem(0);
            }
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mNumArtifacts = getNumArtifacts();
    SmartDashboard.putNumber("artifacts", mNumArtifacts);
  }
}
