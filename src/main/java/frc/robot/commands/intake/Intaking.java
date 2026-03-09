package frc.robot.commands.intake;
import frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


public class Intaking extends Command{
    private final intake m_intake;
    private double duration; 
    private final Timer m_Timer;

    public Intaking(intake intakeSubsystem, double duration){
        this.m_intake = intakeSubsystem;
        m_Timer = new Timer();
        this.duration = duration;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_Timer.reset();
        m_Timer.start();
    }

    @Override
    public void execute() {
        m_intake.changeIntakeState(true);

    }

    @Override
    public boolean isFinished(){
        return m_Timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted){
        m_intake.changeIntakeState(false);
    }
}
