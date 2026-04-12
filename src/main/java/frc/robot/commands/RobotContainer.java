package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser; // I fixed the error by running AutoBuilder.buildAutoChooser() 
                                                        // after registering commands - Gaocan

    public RobotContainer(){
        NamedCommands.registerCommand("runIntake", Intake.getInstance().runIntakeCommand());
        NamedCommands.registerCommand("stopIntake", Intake.getInstance().stopIntakeCommand());
        NamedCommands.registerCommand("lowerPivot", Intake.getInstance().lowerPivot());
        NamedCommands.registerCommand("shoot", Shooter.getInstance().shootInAuto(0.5));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
