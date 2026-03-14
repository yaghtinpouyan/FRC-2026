package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.idConstants;
import frc.robot.constants.velocityMap;
import frc.robot.subsystems.automations.AutoAlign;
import frc.robot.constants.angleMap;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class Shooter extends SubsystemBase{
  private static Shooter ballShooter = null;

  private TalonFX Lshooter1;
  private TalonFX Lshooter2;
  private TalonFX Rshooter1;
  private TalonFX Rshooter2;
  private TalonFX kickerMotor;
  private SparkMax hoodMotor;

  private SparkMaxConfig hoodMotorConfig;
  Follower followLFShooter;
  Follower followRFShooter;
  Follower followLShooter;
  
  private AutoAlign align = AutoAlign.getInstance();
  private Intake ballIntake = Intake.getInstance();
  private Telemetry telemetry;
  private angleMap shootingAMap;
  private velocityMap shootingVMap;
  public double shootingVolts = 3.5;

  private Shooter() {
    //Motor inits
    Lshooter1 = new TalonFX(idConstants.krakenx60_S2);
    Lshooter2 = new TalonFX(idConstants.krakenx60_S3);
    Rshooter1 = new TalonFX(idConstants.krakenx60_S4);
    Rshooter2 = new TalonFX(idConstants.krakenx60_S5);
    kickerMotor = new TalonFX(idConstants.faclon500_S1);
    hoodMotor = new SparkMax(idConstants.neo550_S6, MotorType.kBrushless);


    
    //Motor configs
    hoodMotorConfig = new SparkMaxConfig();
    hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followLFShooter = new Follower(Lshooter1.getDeviceID(), MotorAlignmentValue.Aligned);
    followRFShooter = new Follower(Rshooter1.getDeviceID(), MotorAlignmentValue.Aligned);
    followLShooter = new Follower(Lshooter1.getDeviceID(), MotorAlignmentValue.Opposed);
    Lshooter2.setControl(followLFShooter);
    Rshooter1.setControl(followLShooter);
    Rshooter2.setControl(followRFShooter);
  }

  private double getVirtualTarget(Distance hubDistance){
    telemetry = Telemetry.getInstance();
    ChassisSpeeds chassisVel = telemetry.currentVelocity;
    double xVel = chassisVel.vxMetersPerSecond;
    double yVel = chassisVel.vyMetersPerSecond;
    double flightTime = hubDistance.in(Meters)*0.4;

    double virtualTargetX = hubDistance.in(Meters) - (xVel*flightTime);
    double virtualTargetY = 0 - (yVel * flightTime);
    return Math.hypot(virtualTargetX, virtualTargetY);
  }
  
  public AngularVelocity getFlyWheelVel(){
    double targetDist = getVirtualTarget(align.getHubDist()); 
    shootingVMap = velocityMap.getInstance();
    AngularVelocity targetVelocity = RotationsPerSecond.of(shootingVMap.mainMap.get(targetDist));
    return targetVelocity;
  }

  public Angle getHoodAngle(){
    double targetDist = getVirtualTarget(align.getHubDist());
    shootingAMap = angleMap.getInstance();
    Angle targetAngle = Degrees.of(shootingAMap.mainMap.get(targetDist));
    return targetAngle;
  }

  public void shooterInputManager(double fire, boolean charge){ 
    if(charge){
        kickerMotor.setVoltage(6);
        ballIntake.runIndexer();
    }

    if(fire > 0.1){
        Lshooter1.setVoltage(-shootingVolts);
    }  
    else{
        Lshooter1.set(0);
        ballIntake.stopIndexer();
        kickerMotor.set(0);
    }
  }

  public static Shooter getInstance(){
        if (ballShooter == null){
            ballShooter = new Shooter();
        }
        return ballShooter;
    }
}