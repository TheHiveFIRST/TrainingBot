package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfig;


public class ShooterSubsystem extends SubsystemBase{
    
//Variables 
private SparkMax mShooterLeader; 
private SparkMax mShooterFollower;
private SparkMax mPivotMotor; 
private AbsoluteEncoder mShooterEncoder; 
private PIDController mPivotPID; 

private double pivotOutput = 0;

 //constructor
 public ShooterSubsystem() {
    mShooterEncoder = mPivotMotor.getAbsoluteEncoder();
    mShooterLeader = new SparkMax(Constants.ShooterConstants.SHOOTER_LEADER_CAN_ID, MotorType.kBrushless);
    mShooterFollower = new SparkMax(Constants.ShooterConstants.SHOOTER_FOLLOWER_CAN_ID, MotorType.kBrushless); 
    
    //apply configuration to SparkMax motor controllers
    mShooterLeader.configure(ShooterConfig.shooterLeaderConfig, 
                            ResetMode.kResetSafeParameters, 
                            PersistMode.kPersistParameters);
    mShooterFollower.configure(ShooterConfig.shooterFollowerConfig, 
                            ResetMode.kResetSafeParameters,
                            PersistMode.kPersistParameters);
    mPivotMotor.configure(ShooterConfig.shooterHoodConfig,
                            ResetMode.kResetSafeParameters, 
                            PersistMode.kPersistParameters);
    
    mPivotPID = new PIDController(Constants.ShooterConstants.PIVOT_KP, Constants.ShooterConstants.PIVOT_KI, Constants.ShooterConstants.PIVOT_KD); //organize constants --> shooter constants class 
    }
    
//methods 
public void runShooterSystem(double speed)
{
    mShooterLeader.set(speed);
}

public void setPivotPower(double pivotPower) //need more clear name for this method
{
    mPivotMotor.set(pivotPower);
}

public void pivotControlPID(double targetAngle) //fix 
{
    pivotOutput = mPivotPID.calculate(mShooterEncoder.getPosition(), targetAngle);
    mPivotMotor.set(pivotOutput);
}

public double getValueEncoder() 
{
    return mShooterEncoder.getPosition();
}


//command to set shooter to [xyz] angle

public Command setClosePosition() 
{
    return run(
    () -> {
        pivotControlPID(Constants.ShooterConstants.CLOSE_ANGLE);
    });
}

}
