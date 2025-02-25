package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;
import java.util.function.DoubleSupplier;

public class RollerCommand extends Command {
    private final DoubleSupplier forward;
    private final DoubleSupplier reverse;
    private final RollerSubsystem rollerSubsystem;
    public RollerCommand(DoubleSupplier forward, DoubleSupplier reverse, RollerSubsystem rollerSubsystem){
        this.forward = forward;
        this.reverse = reverse;
        this.rollerSubsystem = rollerSubsystem;
        addRequirements(this.rollerSubsystem);
    }    

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        rollerSubsystem.runRoller(forward.getAsDouble(), reverse.getAsDouble());
    }

    @Override
    public void end (boolean isInterrupted){
        rollerSubsystem.runRoller(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
