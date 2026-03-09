package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.climb;
import frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ChargeShooter;
import frc.robot.commands.shooter.FireShooter;
import frc.robot.commands.climb.climbUp;
import frc.robot.commands.climb.climbDown;
// Intaking class exists upstream but we use a StartEndCommand instead for PathPlanner control

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  //Commands
  ChargeShooter cShooter;
  FireShooter fShooter;
  climbDown cDown;
  climbUp cUp;
  // Use a StartEndCommand for intake to allow PathPlanner to start/stop it without a fixed duration
  StartEndCommand intakeHold;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Register named commands with PathPlanner so events in PathPlanner can trigger subsystem actions.
    // Provide explicit commands that mirror subsystem behavior (on/off, charge/fire, climb up/down).

  // Initialize command instances (use upstream command classes where available)
  // Durations are defaults and can be tweaked later
  double chargeDuration = 2.0; // seconds
  double fireDuration = 0.5; // seconds
    // Create a StartEndCommand for intake: start will pivot and run intake/hopper; end stops them.
    intakeHold = new StartEndCommand(() -> {
      intake i = intake.getInstance();
      i.setIntakePivot();
      i.runIntake(true);
      i.runHopper(true);
    }, () -> {
      intake i = intake.getInstance();
      i.runIntake(false);
      i.runHopper(false);
    }, intake.getInstance());
  cShooter = new ChargeShooter(shooter.getInstance(), chargeDuration);
  fShooter = new FireShooter(shooter.getInstance(), intake.getInstance(), fireDuration);
  cUp = new climbUp(climb.getInstance());
  cDown = new climbDown(climb.getInstance());

  // Register named commands with PathPlanner using upstream Command implementations
  NamedCommands.registerCommand("intake_on", intakeHold);
  // intake_off will stop intake/hopper via immediate call
  NamedCommands.registerCommand("intake_off",
    new InstantCommand(() -> {
      intake i = intake.getInstance();
      i.runIntake(false);
      i.runHopper(false);
    }, intake.getInstance()));

  NamedCommands.registerCommand("climb_up", cUp);
  NamedCommands.registerCommand("climb_down", cDown);

  NamedCommands.registerCommand("shoot_charge", cShooter);
    // Register shoot_fire as a start/end so PathPlanner can control the exact firing window.
    NamedCommands.registerCommand("shoot_fire",
        new StartEndCommand(() -> {
          shooter s = shooter.getInstance();
          intake i = intake.getInstance();
          s.runKicker(true);
          i.setShootingPivot();
        }, () -> shooter.getInstance().runKicker(false), shooter.getInstance(), intake.getInstance()));

  // For backwards compatibility, register simple names mapping to the main actions
  NamedCommands.registerCommand("intake", NamedCommands.getCommand("intake_on"));
  NamedCommands.registerCommand("climb", NamedCommands.getCommand("climb_up"));
  NamedCommands.registerCommand("shoot", NamedCommands.getCommand("shoot_fire"));
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public Command shoot(){
    return new SequentialCommandGroup(cShooter, fShooter);
  }

  public Command intake(){
    return intakeHold;
  }

  public Command climb(){
    return new SequentialCommandGroup(cUp, cDown);
  }
}
