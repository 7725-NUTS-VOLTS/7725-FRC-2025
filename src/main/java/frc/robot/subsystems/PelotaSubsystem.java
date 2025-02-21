package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PelotaConstants;


public class PelotaSubsystem extends SubsystemBase{
    private final SparkMax pMotor;

    public PelotaSubsystem(){
        pMotor = new SparkMax(PelotaConstants.PELOTA_MOTOR_ID, MotorType.kBrushless);
        pMotor.setCANTimeout(250);
        SparkMaxConfig PelotaConfig = new SparkMaxConfig();
        PelotaConfig.voltageCompensation(PelotaConstants.PELOTA_MOTOR_VOLTAGE_COMP);
        PelotaConfig.smartCurrentLimit(PelotaConstants.PELOTA_MOTOR_CURRENT_LIMIT);
        pMotor.configure(PelotaConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic(){

    }

    public void runPelota (double forwardSpin, double reverseSpin){
        pMotor.set(forwardSpin-reverseSpin);
    }
}