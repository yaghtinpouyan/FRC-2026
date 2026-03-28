// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotContainer;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Drive;

public class Robot extends LoggedRobot {
  public Drive drivetrain;
  public OperatorInterface oi;
  // public shooter ballShooter;
  // public intake ballIntake;
  // public climb climber;
  // public autoAlign align;
  public RobotContainer rc;
  Command getAutonomousCommand;

  private PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

////teetettlaksdlask
  public Robot() {
    Logger.recordMetadata("REBUILT-2026", "FRC-6632"); // Set a metadata value

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    drivetrain = Drive.getInstance();
    oi = OperatorInterface.getInstance();
    // ballIntake = intake.getInstance();
    // align = autoAlign.getInstance();
    // ballShooter = shooter.getInstance();
    // climber = climb.getInstance();
    rc = new RobotContainer();
    getAutonomousCommand = rc.getAutonomousCommand();
  }

  @Override
  public void robotInit() {
    //DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // PDH logging
    Logger.recordOutput("PDH/TotalCurrent", pdh.getTotalCurrent());
    Logger.recordOutput("PDH/TotalPower", pdh.getTotalPower());
    Logger.recordOutput("PDH/TotalEnergy", pdh.getTotalEnergy());
    Logger.recordOutput("PDH/Voltage", pdh.getVoltage());
    Logger.recordOutput("PDH/Temperature", pdh.getTemperature());

    // Per-channel current (0-20 for PDH)
    for (int i = 0; i <= 20; i++) {
        Logger.recordOutput("PDH/Channel/" + i, pdh.getCurrent(i));
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    getAutonomousCommand = rc.getAutonomousCommand();
    getAutonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (getAutonomousCommand != null){
      getAutonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
