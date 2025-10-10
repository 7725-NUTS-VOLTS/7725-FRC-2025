package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;


public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore; 
    private Timer dontSeeTagTimer, stopTimer;
    private Swerve drivebase;
    private double tagID =-1;

    public AlignToReefTagRelative(boolean isRightScore, Swerve drivebase){
        xController = new PIDController(0, 0, 0); //AGREGAR CONSTANTES
        yController = new PIDController(0, 0, 0); //AGREGAR CONSTANTES
        rotController = new PIDController(0, 0, 0); //AGREGAR CONSTANTES

        this.isRightScore = isRightScore;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override

    public void initialize(){
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(0); //AGREGAR CONSTANTES
        rotController.setTolerance(0); //AGREGAR CONSTANTES

        xController.setSetpoint(0); //AGREGAR CONSTANTES
        xController.setTolerance(0); //AGREGAR CONSTANTES

        yController.setSetpoint(isRightScore ? 0 : 0); //AGREGAR CONSTANTES
        yController.setTolerance(0); //AGREGAR CONSTANTES

        tagID = LimelightHelpers.getFiducialID("");
    }

    @Override
    public void execute(){
        if(LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") ==tagID){
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            SmartDashboard.putNumber("x", positions[2]);

            double xSpeed = xController.calculate(positions[2]);
            SmartDashboard.putNumber("xspee", xSpeed);
            double ySpeed = -yController.calculate(positions[0]);
            double rotValue = -rotController.calculate(positions[4]);

            drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false, false);

            if(!rotController.atSetpoint() ||
                !yController.atSetpoint() ||
                !xController.atSetpoint()){

            stopTimer.reset();
            }else{
                drivebase.drive(new Translation2d(), 0, false, false);
            }

            SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
        }
    }
        @Override
        public void end(boolean interrupted){
            drivebase.drive(new Translation2d(), 0, false, false);

        }

        @Override
        public boolean isFinished(){
            return this.dontSeeTagTimer.hasElapsed(0) || //AGREGAR CONSTANTES
                stopTimer.hasElapsed(0); //AGREGAR CONSTANTES
        }
    
    
    

}
