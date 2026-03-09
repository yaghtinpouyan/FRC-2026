package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.climb;
import frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Register named commands with PathPlanner so events in PathPlanner can trigger subsystem actions.
    // Provide explicit commands that mirror subsystem behavior (on/off, charge/fire, climb up/down).

    // Intake on: pivot to intake, run intake and hopper forward
    NamedCommands.registerCommand("intake_on",
        new InstantCommand(() -> {
          intake i = intake.getInstance();
          i.setIntakePivot();
          i.runIntake(true);
          i.runHopper(true);
        }, intake.getInstance()));

    // Intake off: stop intake and hopper
    NamedCommands.registerCommand("intake_off",
        new InstantCommand(() -> {
          intake i = intake.getInstance();
          i.runIntake(false);
          i.runHopper(false);
        }, intake.getInstance()));

    // Climb up/down
    NamedCommands.registerCommand("climb_up",
        new InstantCommand(() -> climb.getInstance().setArmPos(true), climb.getInstance()));

    NamedCommands.registerCommand("climb_down",
        new InstantCommand(() -> climb.getInstance().setArmPos(false), climb.getInstance()));

    // Shooter charge (spin up + set hood) and fire (kick)
    NamedCommands.registerCommand("shoot_charge",
        new InstantCommand(() -> shooter.getInstance().shooterInputManager(true, 0.0), shooter.getInstance()));

    NamedCommands.registerCommand("shoot_fire",
        new InstantCommand(() -> shooter.getInstance().shooterInputManager(false, 1.0), shooter.getInstance()));

    // For backwards compatibility, register simple names mapping to the main actions
    NamedCommands.registerCommand("intake", NamedCommands.getCommand("intake_on"));
    NamedCommands.registerCommand("climb", NamedCommands.getCommand("climb_up"));
    NamedCommands.registerCommand("shoot", NamedCommands.getCommand("shoot_fire"));
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }
}
