package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;

public class ElevatorMove extends Command {
	private static ElevatorSubsystem elevator;
	private double power;
	private final Joystick controller;
	private final SlewRateLimiter limiter = new SlewRateLimiter(0.5);

	public ElevatorMove(double power,Joystick controller) {
		this.power = power;
		this.controller = controller;
		elevator = ElevatorSubsystem.getInstance();
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {

		int pov  = controller.getPOV();
		double desiredPower = 0.0;
		if (pov == 0){
			desiredPower = 0.21;
		}else if(pov == 180){
			desiredPower = -0.21;
		}
		double smoothPower = limiter.calculate(desiredPower);
		elevator.setPower(smoothPower);
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