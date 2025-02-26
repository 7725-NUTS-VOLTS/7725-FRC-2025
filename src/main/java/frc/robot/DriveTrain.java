package frc.robot;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrain {
    public static final double kMaxSpeed =3.0;
    public static final double kMaxAngularSpeed = Math.PI;

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation= new Translation2d(0.381, -0.381);
    private final Translation2d m_backfrontLocation = new Translation2d(-.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-.381, -0.381);

    private final SwerveModule m_frontLeft = new SwerveModule(1, null);
    private final SwerveModule m_frontRight = new SwerveModule(2, null);
    private final SwerveModule m_backLeft = new SwerveModule(3, null);
    private final SwerveModule m_backRight = new SwerveModule(4, null);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_frontLeftLocation,m_frontRightLocation, m_backRightLocation, m_backfrontLocation);


    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(m_Kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[]{
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    public DriveTrain(){
        m_gyro.reset();
    }

    public void drive (double xSpeed, Double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
    }
}
