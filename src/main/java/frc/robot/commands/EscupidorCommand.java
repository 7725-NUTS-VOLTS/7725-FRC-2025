package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EscupidorSubSystem;
import java.util.function.DoubleSupplier;

public class EscupidorCommand extends Command {
    private final DoubleSupplier  forward;
    private final DoubleSupplier reverse;
    private final EscupidorSubSystem EscupidorSubsystem;
    public EscupidorCommand(DoubleSupplier forward, DoubleSupplier reverse, EscupidorSubSystem esccupidorSubSystem){
        this.forward = forward;
        this.reverse = reverse;
        this.EscupidorSubsystem = esccupidorSubSystem;
        addRequirements(this.EscupidorSubsystem);
    }
    @Override 
    public void initialize(){
    }

    @Override
    public void execute(){
        EscupidorSubsystem.runRoller(forward.getAsDouble(), reverse.getAsDouble());
    }

    @Override
    public void end (boolean isInterrupted){
        EscupidorSubsystem.runRoller(0,0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
