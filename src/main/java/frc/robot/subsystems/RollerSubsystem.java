package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.RollerConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
    private final SparkMax rMotor;
   
    public RollerSubsystem(){
        rMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        rMotor.setCANTimeout(250);
        SparkMaxConfig RollerConfig = new SparkMaxConfig();
        RollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
        RollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
        rMotor.configure(RollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

    @Override
    public void periodic(){
    }

    public void runRoller (double forward, double reverse){
        rMotor.set(forward-reverse);

    }


}
