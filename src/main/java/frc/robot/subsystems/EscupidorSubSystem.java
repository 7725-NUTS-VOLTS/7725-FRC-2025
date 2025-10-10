package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EscupidorConstants;

public class EscupidorSubSystem extends SubsystemBase {
    private final SparkMax rMotor;
    private final SparkMax fMotor;
    public EscupidorSubSystem(){
        rMotor= new SparkMax(EscupidorConstants.ROLLER_MOTOR_ID_R, MotorType.kBrushless);
        fMotor= new SparkMax(EscupidorConstants.ROLLER_MOTOR_ID_F, MotorType.kBrushless);
        rMotor.setCANTimeout(250);
        fMotor.setCANTimeout(250);
        SparkMaxConfig RollerConfig = new SparkMaxConfig();
        RollerConfig.voltageCompensation(EscupidorConstants.ROLLER_MOTOR_VOLTAGE_COMP);
        RollerConfig.smartCurrentLimit(EscupidorConstants.ROLLER_MOTOR_CURRENT_LIMIT);
        rMotor.configure(RollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        fMotor.configure(RollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

    }
    

    @Override
    public void periodic(){



    }
    public void runRoller(double forward, double reverse){
rMotor.set(reverse);
fMotor.set(forward);


    }
}
