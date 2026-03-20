package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.idConstants;
import frc.robot.constants.velocityMap;
import frc.robot.subsystems.automations.AutoAlign;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import frc.robot.constants.angleMap;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;


public class Shooter extends SubsystemBase{
  private static Shooter ballShooter = null;

  private TalonFX Lshooter1;
  private TalonFX Lshooter2;
  private TalonFX Rshooter1;
  private TalonFX Rshooter2;
  private TalonFX kickerMotor;
  
  private AutoAlign align = AutoAlign.getInstance();
  private Intake ballIntake = Intake.getInstance();
  private Telemetry telemetry;
  private velocityMap shootingVMap;
  public double shootingVolts = 3.5;

  TalonFXConfiguration config1;
  TalonFXConfiguration config2;
  TalonFXConfiguration config3;
  TalonFXConfiguration config4;

  private double shootingInc = 100;
  private double voltInc = 0.15;
  private SmartMotorControllerConfig LmotorConfig;
  private SmartMotorControllerConfig RmotorConfig;

  private SmartMotorController shooterMotor1;
  private SmartMotorController shooterMotor2;
  private SmartMotorController shooterMotor3;
  private SmartMotorController shooterMotor4;

  private FlyWheelConfig shooterConfig;
  private FlyWheel mainShooter;
  //public double startingVal = 1000;
  public double adjustVolts = 3.5;
  //private AngularVelocity startingVel = RotationsPerSecond.of(startingVal);

  private Shooter() {
    //Motor inits
    Lshooter1 = new TalonFX(idConstants.krakenx60_S2);
    Lshooter2 = new TalonFX(idConstants.krakenx60_S3);
    Rshooter1 = new TalonFX(idConstants.krakenx60_S4);
    Rshooter2 = new TalonFX(idConstants.krakenx60_S5);
    kickerMotor = new TalonFX(idConstants.faclon500_S1);;

    //YAMS
    LmotorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(0.000001, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(2500))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
    .withIdleMode(MotorMode.COAST)
    .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW)
    .withStatorCurrentLimit(Amps.of(40))
    .withMotorInverted(true)  //Inverted due to shooter orientation
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withControlMode(ControlMode.CLOSED_LOOP);

    RmotorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(0.000001, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(3000))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
    .withIdleMode(MotorMode.COAST)
    .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW)
    .withStatorCurrentLimit(Amps.of(40))
    .withMotorInverted(false) 
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    .withFeedforward(new SimpleMotorFeedforward(0.001, 0.01, 0.001))
    .withControlMode(ControlMode.CLOSED_LOOP);

    shooterMotor4 = new TalonFXWrapper(Rshooter2, DCMotor.getKrakenX60(1), RmotorConfig);
    shooterMotor2 = new TalonFXWrapper(Lshooter2, DCMotor.getKrakenX60(1), LmotorConfig);
    shooterMotor3 = new TalonFXWrapper(Rshooter1, DCMotor.getKrakenX60(1), RmotorConfig.withLooselyCoupledFollowers(shooterMotor4));
    shooterMotor1 = new TalonFXWrapper(Lshooter1, DCMotor.getKrakenX60(1), LmotorConfig.withLooselyCoupledFollowers(shooterMotor2, shooterMotor3));    

    shooterConfig = new FlyWheelConfig(shooterMotor1)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(4.5))
    .withTelemetry("ShooterWheel", TelemetryVerbosity.LOW)
    .withSoftLimit(RPM.of(-6065), RPM.of(6065));

    mainShooter = new FlyWheel(shooterConfig);

    config1 = new TalonFXConfiguration();
    config2 = new TalonFXConfiguration();
    config3 = new TalonFXConfiguration();
    config4 = new TalonFXConfiguration();

    config1.CurrentLimits.StatorCurrentLimit = 40;
    config2.CurrentLimits.StatorCurrentLimit = 40;
    config3.CurrentLimits.StatorCurrentLimit = 40;
    config4.CurrentLimits.StatorCurrentLimit = 40;
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

  public void setFlywheelVel(AngularVelocity vel){
    mainShooter.setSpeed(vel);
  }

  public void mapIncrementation(boolean up, boolean down){
    if(up) adjustVolts += voltInc;
    if(down) adjustVolts -= voltInc;
  }

  public double getCalcVoltage(Distance distance){
    return velocityMap.getInstance().mainMap.get(distance.baseUnitMagnitude());
  }

  public void shooterInputManager(double charge, boolean fire, boolean up, boolean down){
    if(fire){
        kickerMotor.setVoltage(10);
        ballIntake.runIndexer();
    }
    if(charge > 0.1){
        shooterMotor1.setVoltage(Volts.of(getCalcVoltage(align.getHubDist())));
        //kickerMotor.setVoltage(6);
    }  
    else{
        shooterMotor1.setVoltage(Volts.of(0));
        ballIntake.stopIndexer();
        kickerMotor.set(0);
    }
    mapIncrementation(up, down);
    SmartDashboard.putNumber("Shooter Volts :", adjustVolts);
  }

  public static Shooter getInstance(){
        if (ballShooter == null){
            ballShooter = new Shooter();
        }
        return ballShooter;
    }
}