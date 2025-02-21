package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BrazoConstants;

public class BrazoSubsystem extends SubsystemBase{
    private final SparkMax bMotor;
    private RelativeEncoder encoder;

    public BrazoSubsystem(){
        bMotor = new SparkMax(BrazoConstants.BRAZO_MOTOR_ID, MotorType.kBrushless);
        bMotor.setCANTimeout(250);
        encoder = bMotor.getEncoder();
        SparkMaxConfig BrazoConfig = new SparkMaxConfig();
        BrazoConfig.idleMode(IdleMode.kBrake);
        BrazoConfig.voltageCompensation(BrazoConstants.BRAZO_MOTOR_VOLTAGE_COMP);
        BrazoConfig.smartCurrentLimit(BrazoConstants.BRAZO_MOTOR_CURRENT_LIMIT);
        BrazoConfig.softLimit.reverseSoftLimit(-50).reverseSoftLimitEnabled(true);
        encoder.setPosition(0);
        bMotor.configure(BrazoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void periodic(){
      
    }

    public void runBrazo (double lower, double rise){
        bMotor.set(rise-lower);
    }
}
