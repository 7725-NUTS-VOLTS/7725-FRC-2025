package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevaotrConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance = null;

    private SparkMax masterMotor, followerMotor;
    private SparkMaxConfig masterMotorConfig, followerMotorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    private static double defaultPose = 0;

    private ElevatorSubsystem(){
        masterMotor = new SparkMax(ElevaotrConstants.ELEVATOR_MASTER_MOTOR_ID, MotorType.kBrushless) ; // ADD MOTORID 28
        followerMotor = new SparkMax(ElevaotrConstants.ELEVATOR_FOLLOW_MOTOR_ID, MotorType.kBrushless); // ADD MOTORID

        closedLoopController = masterMotor.getClosedLoopController();

        masterMotorConfig = new SparkMaxConfig();
        followerMotorConfig = new SparkMaxConfig();

        masterMotorConfig.idleMode(IdleMode.kCoast);
        followerMotorConfig.idleMode(IdleMode.kCoast);

        masterMotorConfig.smartCurrentLimit(ElevaotrConstants.ELEVATOR_CURRENT_LIMIT); //ADD CONSTANTS
        followerMotorConfig.smartCurrentLimit(ElevaotrConstants.ELEVATOR_CURRENT_LIMIT); //ADD CONSTANTS

        masterMotorConfig.encoder
            .positionConversionFactor(
                2* Math.PI * ElevaotrConstants.ELEVATOR_ROLLER_RAIDUS/ElevaotrConstants.ELEVATOR_CONVERSION_FACTOR)
            .velocityConversionFactor(
                2*Math.PI * ElevaotrConstants.ELEVATOR_ROLLER_RAIDUS/ElevaotrConstants.ELEVATOR_CONVERSION_FACTOR);

        masterMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ElevaotrConstants.ELEVATOR_P) //ADD CONSTANTS
            .i(ElevaotrConstants.ELEVATOR_I) // ADD CONSTANTS
            .d(ElevaotrConstants.ELEVATOR_D); //ADD CONSTANTS

        masterMotorConfig.closedLoop.maxMotion
            .maxVelocity(ElevaotrConstants.ELEVATOR_MAX_VELO) //ADD CONSTANTS
            .maxAcceleration(ElevaotrConstants.ELEVATOR_MAX_ACCELLERATION) //ADD CONSTANTS
            .allowedClosedLoopError(ElevaotrConstants.ELEVATOR_POSITION_TOLERANCE); //ADD CONSTANTS

        masterMotorConfig.inverted(ElevaotrConstants.Elevator_INVERTED); //ADD CONSTANTS
        //followerMotorConfig.inverted(ElevaotrConstants.Elevator_INVERTED); //ADD CONSTANTS

        masterMotor.configure(masterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotorConfig.follow(masterMotor, true);
        followerMotor.configure(followerMotorConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = masterMotor.getEncoder();

        routine = new SysIdRoutine(
           new SysIdRoutine.Config(null, Voltage.ofBaseUnits(4, Volts), null),
           new SysIdRoutine.Mechanism(
            masterMotor::setVoltage,
            log -> {

                log.motor("elevator")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        masterMotor.getAppliedOutput(),Volts))
                        .linearPosition(m_angle.mut_replace(encoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(encoder.getVelocity(), MetersPerSecond));

            }, this
           )
        );
    }

    public void resetPosition(){
        encoder.setPosition(0);
        defaultPose = 0;
    }

    public void moveElevatotToPose(double point){
        closedLoopController.setReference(point, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

    }

    public void stop(){
        masterMotor.stopMotor();
    }

    public void setPower(double power){
        masterMotor.setVoltage(power*12);
    }

    public boolean isInPoint(double point){
        return(Math.abs(encoder.getPosition()-point) <= ElevaotrConstants.ELEVATOR_POSITION_TOLERANCE); //ADD CONSTANTS
    }

    public double getPose(){
        return encoder.getPosition();
    }

    public void setDefaultPose(double pose){
        defaultPose = pose;
    }

    public static double getDefaultPose(){
        return defaultPose;
    }

    public static ElevatorSubsystem getInstance(){
        if(instance==null){
            instance = new ElevatorSubsystem();
        }
        return instance;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("elevatorPose", encoder.getPosition());
        SmartDashboard.putNumber("elevatorSpeed", encoder.getVelocity());
        SmartDashboard.putNumber("elevatorCurrent", masterMotor.getOutputCurrent());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return routine.dynamic(direction);
    }

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_angle= Centimeters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine routine;
}
