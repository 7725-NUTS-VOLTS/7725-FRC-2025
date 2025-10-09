package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubSystemBase;
import.frc.robot.Constants.EscupidorConstants;
public class EsccupidorSubSystem extends SubSystemBase {
    private final SparkMax rMotor;
    private final SparkMax fMotor;
    public EsccupidorSubSystem(){
        rMotor= new SparkMax(ROLLER_MOTOR_ID_R, MotorType.kBrushLess);
        fMotor= new SparkMax(ROLLER_MOTOR_ID_F, MotorType.kBrushLess);
        rMotor.setCANTimeout(milliseconds:250);
        fMotor.setCANTimeout(milliseconds:250);
        SparkMaxConfig RollerConfig = new SparkMaxConfig();
        RollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
        RollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
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
