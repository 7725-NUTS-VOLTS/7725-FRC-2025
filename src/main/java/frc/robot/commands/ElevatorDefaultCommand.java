package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefaultCommand extends Command {
    private ElevatorSubsystem elevator;
    private double power;

    private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(0, 0);

    private final ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0, m_Constraints);

    ElevatorFeedforward elevatorFeedforward= new ElevatorFeedforward(0, 0, 0);

    public ElevatorDefaultCommand() {
        elevator = ElevatorSubsystem.getInstance();
        this.power = 0;
        addRequirements();

        pidController.setTolerance(0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        power = pidController.calculate(elevator.getPose(), ElevatorSubsystem.getDefaultPose())
        + elevatorFeedforward.calculate(pidController.getSetpoint().velocity);

        if(pidController.getPositionError() < 0 && power < -0.4){
            power = -0.4;
    }

    elevator.setPower(power);
}

@Override
public void end(boolean interrupted) {
    elevator.setPower(0);
}

@Override
public boolean isFinished(){
    return false;
}
}



