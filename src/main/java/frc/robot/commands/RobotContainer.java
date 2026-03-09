package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.*; 
import frc.robot.commands.intake.Intaking;
import frc.robot.commands.shooter.ChargeShooter;
import frc.robot.commands.shooter.FireShooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final drive m_drive = drive.getInstance();
  private final intake m_intake = intake.getInstance();
  private final shooter m_shooter = shooter.getInstance();
  private final climb m_climb = climb.getInstance();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("IntakeBalls", new Intaking(m_intake, 3.0));
    NamedCommands.registerCommand("ChargeShooter", new ChargeShooter(m_shooter, 3.0));
    NamedCommands.registerCommand("Shoot", new FireShooter(m_shooter, m_intake, 6.0));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }
}