package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private Intake intake = Intake.getInstance();
    private Shooter shooter = Shooter.getInstance();


    public RobotContainer(){
        NamedCommands.registerCommand("runIntake", intake.runIntakeCommand());
        NamedCommands.registerCommand("stopIntake", intake.stopIntakeCommand());
        NamedCommands.registerCommand("lowerPivot", intake.lowerPivot());
        NamedCommands.registerCommand("shoot", shooter.shootInAuto(0.5, true));
        NamedCommands.registerCommand("charge", shooter.shootInAuto(0.5, false));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
