package frc.robot.commands;

import frc.robot.subsystems.PelotaSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class PelotaCommand extends Command{
    private final DoubleSupplier forward;
    private final DoubleSupplier reverse;
    private final PelotaSubsystem pelotaSubsystem;
    public PelotaCommand(DoubleSupplier forward, DoubleSupplier reverse, PelotaSubsystem pelotaSubsystem){
        this.forward = forward;
        this.reverse = reverse;
        this.pelotaSubsystem = pelotaSubsystem;
        addRequirements(this.pelotaSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        pelotaSubsystem.runPelota(forward.getAsDouble(), reverse.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted){
        pelotaSubsystem.runPelota(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

