package frc.robot.configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;


public final class ShooterConfig {
    public static final SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterHoodConfig = new SparkMaxConfig();

    static {
        //set current limits for flywheels 
        shooterLeaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);
        shooterFollowerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

        //set current limits for hood pivot
        shooterHoodConfig 
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

        shooterLeaderConfig.follow(Constants.ShooterConstants.SHOOTER_LEADER_CAN_ID);


    }
}
