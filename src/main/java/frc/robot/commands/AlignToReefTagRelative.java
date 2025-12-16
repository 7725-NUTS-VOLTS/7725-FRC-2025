package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.AlignConstants;;


public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore; 
    private Timer dontSeeTagTimer, stopTimer;
    private Swerve drivebase;
    private double tagID =-1;

    public AlignToReefTagRelative(boolean isRightScore, Swerve drivebase){
        xController = new PIDController(Constants.AlignConstants.X_REEF_ALIGNMENT_P, 0, 0); 
        yController = new PIDController(Constants.AlignConstants.Y_REEF_ALIGNMENT_P, 0, 0); 
        rotController = new PIDController(Constants.AlignConstants.ROT_REEF_ALIGNMENT_P, 0, 0); 

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

        rotController.setSetpoint(Constants.AlignConstants.ROT_SETPOINT_REEF_ALIGNMENT); 
        rotController.setTolerance(Constants.AlignConstants.ROT_TOLERANCE_REEF_ALIGNMENT); 

        xController.setSetpoint(Constants.AlignConstants.X_SETPOINT_REEF_ALIGNMENT); 
        xController.setTolerance(Constants.AlignConstants.X_TOLERANCE_REEF_ALIGNMENT); 

        yController.setSetpoint(isRightScore ? Constants.AlignConstants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.AlignConstants.Y_SETPOINT_REEF_ALIGNMENT); 
        yController.setTolerance(Constants.AlignConstants.Y_TOLERANCE_REEF_ALIGNMENT); 

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
            return this.dontSeeTagTimer.hasElapsed(Constants.AlignConstants.DONT_SEE_TAG_WAIT_TIME) || 
                stopTimer.hasElapsed(Constants.AlignConstants.POSE_VAIDATION_TIME); 
        }
    
    
    

}
