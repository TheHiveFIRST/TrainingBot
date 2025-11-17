package frc.robot.configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import frc.robot.Constants.IntakeConstants;

public final class IntakeConfig {
        public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig feederConfig = new SparkMaxConfig();
        
        static {
            // set current limits for our intake
            rollerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);         
            
            feederConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        }
}
