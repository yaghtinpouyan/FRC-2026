package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb;

public class climbUp extends Command{
    private final climb m_climb;

    public climbUp(climb climbSubsystem){
        this.m_climb = climbSubsystem;
        addRequirements(m_climb);
    }

    @Override
    public void execute() {
        m_climb.climbInputHandler(0);
    }
}