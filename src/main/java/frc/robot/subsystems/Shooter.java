package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.hoodMap;
import frc.robot.constants.idConstants;
import frc.robot.constants.velocityMap;
import frc.robot.subsystems.automations.AutoAlign;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
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

  private SparkMax hoodMotor;
  private RelativeEncoder hoodEncoder;
  private SparkClosedLoopController hoodController;
  
  private AutoAlign align = AutoAlign.getInstance();
  private Intake ballIntake = Intake.getInstance();
  private Telemetry telemetry;
  private velocityMap shootingVMap;
  public double shootingVolts = 3.5;

  private SmartMotorControllerConfig LmotorConfig;
  private SmartMotorControllerConfig RmotorConfig;
  private SmartMotorController shooterMotor1;
  private SmartMotorController shooterMotor2;
  private SmartMotorController shooterMotor3;
  private SmartMotorController shooterMotor4;
  public boolean isShooting = false;

  public double startingVal = 1500;
  private double shootingInc = 50;
  public double startingPos = 20;
  private double posInc = 2.5;

  private Shooter() {
    //Motor inits
    Lshooter1 = new TalonFX(idConstants.krakenx60_S2);
    Lshooter2 = new TalonFX(idConstants.krakenx60_S3);
    Rshooter1 = new TalonFX(idConstants.krakenx60_S4);
    Rshooter2 = new TalonFX(idConstants.krakenx60_S5);
    kickerMotor = new TalonFX(idConstants.faclon500_S1);
    
    hoodMotor = new SparkMax(idConstants.neo550_S6, MotorType.kBrushless);
    hoodEncoder = hoodMotor.getEncoder();
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodController = hoodMotor.getClosedLoopController();

    hoodConfig.encoder.positionConversionFactor(360.0/1250.0)
    .velocityConversionFactor(360.0/1250.0/60.0);

    hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.01)
    .i(0)
    .d(0)
    .outputRange(-1, 1);

    hoodConfig.closedLoop.maxMotion.cruiseVelocity(11000)
    .maxAcceleration(8000)
    .allowedProfileError(0.5)
    .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    
    hoodConfig.inverted(true);
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodEncoder.setPosition(20);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //YAMS
    LmotorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(0.17053, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(3000))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
    .withIdleMode(MotorMode.COAST)
    .withTelemetry("ShooterMotor1", TelemetryVerbosity.LOW)
    .withStatorCurrentLimit(Amps.of(80))
    .withSupplyCurrentLimit(Amps.of(40))
    .withMotorInverted(true)  //Inverted due to shooter orientation
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    .withFeedforward(new SimpleMotorFeedforward(0.11706, 0.12336, 0.06718))
    .withControlMode(ControlMode.CLOSED_LOOP);

    RmotorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(0.17053, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(3000))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
    .withIdleMode(MotorMode.COAST)
    .withTelemetry("ShooterMotor2", TelemetryVerbosity.LOW)
    .withStatorCurrentLimit(Amps.of(80))
    .withSupplyCurrentLimit(Amps.of(40))
    .withMotorInverted(false) 
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    .withFeedforward(new SimpleMotorFeedforward(0.11706, 0.12336, 0.01))
    .withControlMode(ControlMode.CLOSED_LOOP);

    shooterMotor4 = new TalonFXWrapper(Rshooter2, DCMotor.getKrakenX60(1), RmotorConfig);
    shooterMotor2 = new TalonFXWrapper(Lshooter2, DCMotor.getKrakenX60(1), LmotorConfig);
    shooterMotor3 = new TalonFXWrapper(Rshooter1, DCMotor.getKrakenX60(1), RmotorConfig.withLooselyCoupledFollowers(shooterMotor4));
    shooterMotor1 = new TalonFXWrapper(Lshooter1, DCMotor.getKrakenX60(1), LmotorConfig.withLooselyCoupledFollowers(shooterMotor2, shooterMotor3)); 
  }

  public AngularVelocity getFlyWheelVel(){
    double targetDist = getVirtualTarget(align.getHubDist()); 
    shootingVMap = velocityMap.getInstance();
    AngularVelocity targetVelocity = RotationsPerSecond.of(shootingVMap.mainMap.get(targetDist));
    return targetVelocity;
  }

  public void mapIncrementation(boolean up, boolean down){
    if(up) startingVal += shootingInc;
    if(down) startingVal -= shootingInc;
  }

  public void hoodIncrementation(boolean up, boolean down){
    if(up) startingPos += posInc;
    if(down) startingPos -= posInc;
  }

  public double getCalcVoltage(Distance distance){
    return velocityMap.getInstance().mainMap.get(distance.baseUnitMagnitude());
  }

  public void shooterInputManager(double charge, boolean fire, boolean up, boolean down, boolean manual){
    if(!manual) manualShooter(charge, fire, up, down);
    else{
      shooterMap(charge, fire);
    }
  }

  //Flywheel
  public void manualShooter(double charge, boolean fire, boolean up, boolean down){
    if(isAtTargetSpeed()){
        kickerMotor.setVoltage(10);
        ballIntake.runIndexer();
        isShooting = true;
        ballIntake.setPivotAngle(0); 
        //ballIntake.setPivot(0); //Raise pivot when shooting
    }

    if(charge > 0.1){
        //shooterMotor1.setVelocity(RPM.of(velocityMap.getInstance().mainMap.get(align.getHubDist().baseUnitMagnitude())));
        AngularVelocity vel = RPM.of(startingVal);
        shooterMotor1.setVelocity(vel);
        isShooting = true;
    }  
    else{
        shooterMotor1.setVelocity(RPM.of(0));
        ballIntake.stopIndexer();
        kickerMotor.set(0);
        isShooting = false;
    }
    mapIncrementation(up, down);
    SmartDashboard.putNumber("Shooter RPM :", startingVal);
  }

  public void shooterMap(double charge, boolean fire){
    if(isAtTargetMapSpeed()){
      kickerMotor.setVoltage(10);
      ballIntake.runIndexer();
      isShooting = true;
      ballIntake.setPivotAngle(0); 
      //ballIntake.setPivot(0); //Raise pivot when shooting
    }
    if(charge > 0.1){
      isShooting = true;
      shooterMotor1.setVelocity(RPM.of(velocityMap.getInstance().mainMap.get(align.getHubDist().baseUnitMagnitude())));
    }  
    else{
      isShooting = false;
      shooterMotor1.setVelocity(RPM.of(0));
      ballIntake.stopIndexer();
      kickerMotor.set(0);  
    }
  }

  public boolean isAtTargetSpeed(){
    AngularVelocity current = shooterMotor1.getRotorVelocity();
    AngularVelocity targetSpeed = RPM.of(startingVal);
    return Math.abs(current.in(RPM) - targetSpeed.in(RPM)) < Constants.shootingTolerence;
  }

  public boolean isAtTargetMapSpeed(){
    AngularVelocity current = shooterMotor1.getRotorVelocity();
    AngularVelocity targetSpeed = RPM.of(velocityMap.getInstance().mainMap.get(align.getHubDist().baseUnitMagnitude()));
    return Math.abs(current.in(RPM) - targetSpeed.in(RPM)) < Constants.shootingTolerence;
  }

  //Hood
  public void hoodAdjust(){
    double targetAngle = hoodMap.getInstance().mainMap.get(align.getHubDist().baseUnitMagnitude());
    hoodController.setSetpoint(targetAngle, ControlType.kMAXMotionPositionControl);
    SmartDashboard.putNumber("Hood Angle :", targetAngle);
  }


  private double getVirtualTarget(Distance hubDistance){
    telemetry = Telemetry.getInstance();
    ChassisSpeeds chassisVel = telemetry.currentVelocity;
    double xVel = chassisVel.vxMetersPerSecond;
    double yVel = chassisVel.vyMetersPerSecond;
    //Placeholder use actual flight time from the shooter map
    double flightTime = hubDistance.in(Meters)*0.4;

    double virtualTargetX = hubDistance.in(Meters) - (xVel*flightTime);
    double virtualTargetY = 0 - (yVel * flightTime);
    return Math.hypot(virtualTargetX, virtualTargetY);
  }

  public Command shootInAuto(double charge, boolean fire){
    return run(() -> shooterMap(charge, fire));
  }

  public static Shooter getInstance(){
        if (ballShooter == null){
            ballShooter = new Shooter();
        }
        return ballShooter;
    }
}