package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;

public class FireShooter extends Command {
    private final shooter m_shooter;
    private final intake m_intake;

    public FireShooter(shooter shooterSubsystem, intake intakeSubsystem) {
        m_shooter = shooterSubsystem;
        m_intake = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        m_shooter.runKicker(true);
        m_intake.setShootingPivot();
    }

}
