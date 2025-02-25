package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BrazoSubsystem;

public class BrazoCommand  extends Command{
   
    private final DoubleSupplier lower;
    private final DoubleSupplier rise;
    private final BrazoSubsystem brazoSubsystem;
    public BrazoCommand(DoubleSupplier lower, DoubleSupplier rise, BrazoSubsystem brazoSubsystem){
        this.lower = lower;
        this.rise = rise;
        this.brazoSubsystem = brazoSubsystem;
        addRequirements(this.brazoSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        brazoSubsystem.runBrazo(rise.getAsDouble(), lower.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted){
        brazoSubsystem.runBrazo(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

