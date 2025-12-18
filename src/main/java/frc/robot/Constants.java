package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    //TO DO: ADD CONSTANTS LIMELIGHT FUNCTION 

    public static final class Swerve {
        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(28); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 CHECKED*/
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees( 100.458984375); //61.171875  104.23836
            public static final Rotation2d cancoderOffset = Rotation2d.fromRotations(0.293);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, cancoderOffset);
        }

        /* Front Right Module - Module 1  Checked*/
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees( -32.51953125 ); //-25.429688  -27.68544
            public static final Rotation2d cancoderOffset = Rotation2d.fromRotations(-0.087);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, cancoderOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2; 
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(   -28.125);  // -17.22672 -23.5546875
            public static final Rotation2d cancoderOffset = Rotation2d.fromRotations(-0.061);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, cancoderOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 9;    
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees( -164.70703125 ); //-162.949322
            public static final Rotation2d cancoderOffset = Rotation2d.fromRotations(-0.451);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, cancoderOffset);
        }
        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
             new PIDConstants(3.5, 1, 1), 
            new PIDConstants(3.5, 1, 1));
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final class EscupidorConstants{
        public static final int ROLLER_MOTOR_ID_R = 25;
        public static final int ROLLER_MOTOR_ID_F= 32;
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60; 
        public static final double ROLLER_EJECT_VALUE = 0.5;
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    }

    public static final class ElevaotrConstants{
        public static final int ELEVATOR_MASTER_MOTOR_ID = 30;
        public static final int ELEVATOR_FOLLOW_MOTOR_ID = 28;
        public static final double ELEVATOR_P  = 0.15;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.005;
        public static final double ELEVATOR_MAX_ACCELLERATION = 5000;
        public static final double ELEVATOR_MAX_VELO = 3000;
        public static final double ELEVATOR_POSITION_TOLERANCE = 0.2;
        public static final int ELEVATOR_CURRENT_LIMIT = 40;
        public static final int ELEVATOR_ROLLER_RAIDUS = 1;
        public static final int ELEVATOR_CONVERSION_FACTOR = 12;
        public static final Boolean Elevator_INVERTED = true;

    }
    public static final class BrazoConstants{
        public static final int BRAZO_MOTOR_ID =25;
        public static final int BRAZO_MOTOR_CURRENT_LIMIT = 60; 
        public static final double BRAZO_LOWER_VALUE = 0.25;
        public static final double BRAZO_RISE_VALUE = 0.25;
        public static final double BRAZO_MOTOR_VOLTAGE_COMP = 10;
    }
    public static final class AlignConstants{
        public static final double ROT_REEF_ALIGNMENT_P = 0.058;
        public static final double X_REEF_ALIGNMENT_P = 3.3;
        public static final double Y_REEF_ALIGNMENT_P = 3.3;

        public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
        public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;
        public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;

        public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
        public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
        public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;
        public static final double DONT_SEE_TAG_WAIT_TIME = 1;
        public static final double POSE_VAIDATION_TIME = 1;
    }
    
}