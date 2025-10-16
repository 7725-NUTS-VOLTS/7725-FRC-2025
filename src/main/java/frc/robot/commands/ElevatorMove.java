package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMove extends Command {
	private static ElevatorSubsystem elevator;
	private double power;

	public ElevatorMove(double power) {
		this.power = power;

		elevator = ElevatorSubsystem.getInstance();
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		elevator.setPower(this.power);
	}

	@Override
	public void end(boolean interrupted) {
		elevator.stop();
		elevator.setDefaultPose(elevator.getPose());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}