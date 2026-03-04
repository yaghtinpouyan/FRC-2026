package frc.robot.test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter;
import frc.robot.constants.Constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterTest extends SubsystemBase {
    private static ShooterTest shooterTest = null;
    private shooter ballShooter = shooter.getInstance();
    double TestFlyWheelRPM = SmartDashboard.getNumber("RPM", 0);
    AngularVelocity testFlyWheelVelocity = RotationsPerSecond.of(TestFlyWheelRPM/60);
    Angle testHoodAngle = Degrees.of(SmartDashboard.getNumber("Hood Angle", 0));

    public void SetTestVelocity(boolean SetTestVel){
        
        if (SetTestVel) ballShooter.setFlyWheelVel(testFlyWheelVelocity);
    }

    public void setTestHoodAngle(boolean SetTestHoodAngle){
        if(SetTestHoodAngle) ballShooter.setHoodAngle(testHoodAngle);
    }

    public void runFlyWheelSysID(){
        ballShooter.flyWheelSysId(Constants.flyWheelSysIdMaxVoltage, Constants.flyWheelSysIdStep, Constants.flyWheelSysIdDuration);
    }

    public void runHoodSysID(){
        ballShooter.hoodSysId(Constants.hoodSysIdMaxVoltage, Constants.hoodSysIdStep, Constants.hoodSysIdDuration);
    }

    public static ShooterTest getInstance(){
        if (shooterTest == null){
            shooterTest = new ShooterTest();
        }
        return shooterTest;
    }
}
