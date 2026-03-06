package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;

public class ChargeShooter extends Command {
    private final shooter m_shooter;

    public ChargeShooter(shooter shooterSubsystem) {
        m_shooter = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooter.setHoodAngle(m_shooter.getHoodAngle());
        m_shooter.setFlyWheelVel(m_shooter.getFlyWheelVel());
    }

}
