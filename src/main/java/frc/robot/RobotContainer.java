package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.BrazoConstants;
import frc.robot.Constants.PelotaConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick opperator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    //private final int speedAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton robotCentric =
        new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeroGyro1 = 
        new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroGyro2 = 
        new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton resetWheels = 
        new JoystickButton(driver, XboxController.Button.kA.value);

    /*Opperator buttons */
    private final JoystickButton ejectCoral = new JoystickButton(opperator, XboxController.Button.kA.value);
    private final JoystickButton retractCoral = new JoystickButton(opperator, XboxController.Button.kY.value);
    private final JoystickButton catchAlgae = new JoystickButton(opperator, XboxController.Button.kX.value);
    private final JoystickButton spitAlgae = new JoystickButton(opperator, XboxController.Button.kB.value);
    private final JoystickButton bajarBrazo = new JoystickButton(opperator, XboxController.Button.kRightBumper.value);
    private final JoystickButton subirBrazo = new JoystickButton(opperator, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private final PelotaSubsystem pelotaSubsystem = new PelotaSubsystem();
    private final BrazoSubsystem brazoSubsystem = new BrazoSubsystem();

    /*Auto Selector on Dashboard*/
    private SendableChooser<Command> autoChooser;

    /*Field Visualization on Dashboard*/
    private final Field2d field;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("DropCoral", new RollerCommand(()-> 0, ()-> RollerConstants.ROLLER_EJECT_VALUE, rollerSubsystem).withTimeout(2));
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean()
            )
        );

        PathPlannerLogging.setLogCurrentPoseCallback((pose)->{
            field.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose)->{
            field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses)->{
            field.getObject("path").setPoses(poses);
        });

        //* driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1)
        //* driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1)
        //* SmartDashboard.getNumber("SpeedLimit", 1) * 0.60

        // Configure the button bindings
        configureButtonBindings();
        autoChooser  = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro1.and(zeroGyro2).onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        resetWheels.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        /* Operator Buttons */
        ejectCoral.whileTrue(new RollerCommand(()->0, ()->RollerConstants.ROLLER_EJECT_VALUE, rollerSubsystem));
        retractCoral.whileTrue(new RollerCommand(()->RollerConstants.ROLLER_EJECT_VALUE, ()->0, rollerSubsystem));
        catchAlgae.whileTrue(new PelotaCommand(()->PelotaConstants.PELOTA_SPIN_VALUE, ()->0, pelotaSubsystem));
        spitAlgae.whileTrue(new PelotaCommand(()->0, ()->PelotaConstants.PELOTA_SPIN_VALUE, pelotaSubsystem));
        bajarBrazo.whileTrue(new BrazoCommand(()->0,()->BrazoConstants.BRAZO_LOWER_VALUE , brazoSubsystem));
        subirBrazo.whileTrue(new BrazoCommand(()->BrazoConstants.BRAZO_RISE_VALUE,()->0 , brazoSubsystem));
    }   

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Will run in autonomous
        return autoChooser.getSelected();
    }
}