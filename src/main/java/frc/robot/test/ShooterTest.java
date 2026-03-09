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
    double testHoodInput = 0;
    
    double testFlyWheelRPM = 0;
    AngularVelocity testFlyWheelVelocity = RotationsPerSecond.of(testFlyWheelRPM/60);
    Angle testHoodAngle = Degrees.of(testHoodInput);

    private ShooterTest() {
        SmartDashboard.putNumber("HoodAngle", 0.0);
        testHoodInput = SmartDashboard.getNumber("HoodAngle", 0);

        SmartDashboard.putNumber("RPM", 0.0);
        testFlyWheelRPM = SmartDashboard.getNumber("RPM", 0.0);
    }
    

    public void SetTestVelocity(boolean SetTestVel){
        
        if (SetTestVel) ballShooter.setFlyWheelVel(testFlyWheelVelocity);
    }

    public void setTestHoodAngle(boolean SetTestHoodAngle){
        if(SetTestHoodAngle) ballShooter.setHoodAngle(testHoodAngle);
    }

    public void runFlyWheelSysID(boolean run){
        if(run) ballShooter.flyWheelSysId(Constants.flyWheelSysIdMaxVoltage, Constants.flyWheelSysIdStep, Constants.flyWheelSysIdDuration);
    }

    public void runHoodSysID(boolean run){
        if(run) ballShooter.hoodSysId(Constants.hoodSysIdMaxVoltage, Constants.hoodSysIdStep, Constants.hoodSysIdDuration);
    }

    public static ShooterTest getInstance(){
        if (shooterTest == null){
            shooterTest = new ShooterTest();
        }
        return shooterTest;
    }
}
