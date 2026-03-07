package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ChargeShooter;
import frc.robot.commands.shooter.FireShooter;
import frc.robot.commands.climb.climbUp;
import frc.robot.commands.climb.climbDown;
import frc.robot.commands.intake.Intaking;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  //Commands
  ChargeShooter cShooter;
  FireShooter fShooter;
  climbDown cDown;
  climbUp cUp;
  Intaking intakeBalls;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public Command shoot(){
    return new SequentialCommandGroup(cShooter, fShooter);
  }

  public Command intake(){
    return intakeBalls;
  }

  public Command climb(){
    return new SequentialCommandGroup(cUp, cDown);
  }
}
