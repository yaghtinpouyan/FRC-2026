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
import frc.robot.constants.angleMap;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  TalonFXConfiguration config1;
  TalonFXConfiguration config2;
  TalonFXConfiguration config3;
  TalonFXConfiguration config4;

  private double shootingInc = 100;
  private SmartMotorControllerConfig krakenConfig;
  private SmartMotorController shooterMotor;
  private FlyWheelConfig shooterConfig;
  private FlyWheel mainShooter;
  public double startingVal = 3000;
  private AngularVelocity startingVel = RotationsPerSecond.of(startingVal);

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

    //YAMS
    krakenConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(0.0001, 0, 0, RPM.of(6000), RotationsPerSecondPerSecond.of(5513))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
    .withIdleMode(MotorMode.COAST)
    .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW)
    .withStatorCurrentLimit(Amps.of(40))
    .withMotorInverted(true)  //Inverted due to shooter orientation
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.52, 0.14))
    .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.52, 0.14))
    .withControlMode(ControlMode.CLOSED_LOOP);

    shooterConfig = new FlyWheelConfig(shooterMotor)
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

    Lshooter1.getConfigurator().apply(config1);
    Lshooter2.getConfigurator().apply(config2);
    Rshooter1.getConfigurator().apply(config3);
    Rshooter2.getConfigurator().apply(config4);
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

  public void setFlywheelVel(AngularVelocity vel){
    mainShooter.setSpeed(vel);
  }

  public void mapIncrementation(boolean up, boolean down){
    if(up) startingVal += shootingInc;
    if(down) startingVal -= shootingInc;
  }

  public void shooterInputManager(double charge, boolean fire, boolean up, boolean down){
    if(fire){
        kickerMotor.setVoltage(6);
        ballIntake.runIndexer();
    }
    if(charge > 0.1){
        setFlywheelVel(startingVel);
        kickerMotor.setVoltage(6);
    }  
    else{
        mainShooter.setSpeed(RotationsPerSecond.of(0));
        ballIntake.stopIndexer();
        kickerMotor.set(0);
    }
    mapIncrementation(up, down);
    SmartDashboard.putNumber("Shooter RPM", startingVal);
  }

  public static Shooter getInstance(){
        if (ballShooter == null){
            ballShooter = new Shooter();
        }
        return ballShooter;
    }
}