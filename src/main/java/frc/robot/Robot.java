// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotContainer;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Drive;

public class Robot extends TimedRobot {
  public Drive drivetrain;
  public OperatorInterface oi;
  // public shooter ballShooter;
  // public intake ballIntake;
  // public climb climber;
  // public autoAlign align;
  // public RobotContainer rc;
  // public Command getAutonomousCommand;


  public Robot() {
    drivetrain = Drive.getInstance();
    oi = OperatorInterface.getInstance();
    // ballIntake = intake.getInstance();
    // align = autoAlign.getInstance();
    // ballShooter = shooter.getInstance();
    // climber = climb.getInstance();
    // rc = new RobotContainer();
  }

  @Override
  public void robotInit() {
    //DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // getAutonomousCommand = rc.getAutonomousCommand();
    
  //   if (getAutonomousCommand != null){
  //     getAutonomousCommand.schedule();
  //  }
  }

  @Override
  public void autonomousPeriodic() {
 
  }

  @Override
  public void teleopInit() {
    // if (getAutonomousCommand != null){
    //   getAutonomousCommand.cancel();
    // }
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
