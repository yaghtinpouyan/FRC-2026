package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.idConstants;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase{
    private static Intake intake = null;

    TalonFX intakeRollers;
    TalonFX indexer;
    SparkMax pivotMotor;
    private Arm pivot;
    private final SmartMotorControllerConfig falconConfig;
    private SmartMotorControllerConfig smcConfig;
    private SmartMotorController indexerSystem;
    private SmartMotorController pivotSmartMotorController;
    private ArmConfig pivotCfg;
    public boolean isIntaking = false;
    
    private Intake(){
        intakeRollers = new TalonFX(idConstants.falcon500_I1);
        indexer = new TalonFX(idConstants.falcon500_I2);
        pivotMotor = new SparkMax(idConstants.neo_I3, MotorType.kBrushless);
    
        falconConfig = new SmartMotorControllerConfig(this)
                            .withClosedLoopController(0.0001, 0, 0, RPM.of(6000), RotationsPerSecondPerSecond.of(100))
                            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 1)))
                            .withIdleMode(MotorMode.COAST)
                            .withStatorCurrentLimit(Amps.of(80))
                            .withSupplyCurrentLimit(Amps.of(40))
                            .withMotorInverted(true)
                            .withClosedLoopRampRate(Seconds.of(0.25))
                            .withOpenLoopRampRate(Seconds.of(0.25))
                            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withControlMode(ControlMode.CLOSED_LOOP);

        smcConfig = new SmartMotorControllerConfig(this)
                            .withControlMode(ControlMode.CLOSED_LOOP)
                            .withClosedLoopController(10.92, 0, 0, RPM.of(6000), DegreesPerSecondPerSecond.of(45))//TODO
                            .withSimClosedLoopController(10.92, 0, 0, RPM.of(6000), DegreesPerSecondPerSecond.of(45))
                            .withFeedforward(new ArmFeedforward(0, 0, 0))
                            .withSimFeedforward(new ArmFeedforward(0, 0, 0))
                            .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
                            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))//TODO
                            .withMotorInverted(false)
                            .withIdleMode(MotorMode.BRAKE)
                            .withStatorCurrentLimit(Amps.of(40))
                            .withClosedLoopRampRate(Seconds.of(0.25))
                            .withOpenLoopRampRate(Seconds.of(0.25));


        pivotSmartMotorController = new SparkWrapper(pivotMotor, DCMotor.getNEO(1), smcConfig);

        pivotCfg = new ArmConfig(pivotSmartMotorController)
        .withSoftLimits(Degrees.of(-20), Degrees.of(10))
        .withStartingPosition(Degrees.of(-5))
        .withLength(Feet.of(3))
        .withMass(Pounds.of(1))
        .withTelemetry("Pivot", TelemetryVerbosity.HIGH);

        // Arm Mechanism
        pivot = new Arm(pivotCfg);

        indexerSystem = new TalonFXWrapper(indexer, DCMotor.getFalcon500(1), falconConfig);
    }

    public void runIndexer(){
        //AngularVelocity vel = RotationsPerSecond.of(6000);
        indexerSystem.setVoltage(Volts.of(10));
    }

    public void stopIndexer(){
        indexer.setVoltage(0);
    }

    public void runRollers(double input1, boolean input2){
        if(input1 > 0.1) {
            intakeRollers.setVoltage(-6);
            isIntaking = true;
        }
        else if(input2){
            intakeRollers.setVoltage(6);
            isIntaking = true;
        }
        else{
            intakeRollers.setVoltage(0);
            isIntaking = false;
        }
    }

    public Angle getAngle(){
        return pivot.getAngle();
        //return pivotSmartMotorController.getMechanismPosition();
    }

    public void setAngle(Angle angle) {
        pivot.run(angle).execute();
        //pivotSmartMotorController.setPosition(angle);
    }

    public void stowIntake(){
        pivot.run(Degrees.of(Constants.intakeAngleDown)).execute();
        //pivotSmartMotorController.setPosition(Degrees.of(Constants.intakeAngleDown));
    }

    public void deployIntake(){
        pivot.run(Degrees.of(Constants.intakeAngleUp)).execute();
        //pivotSmartMotorController.setPosition(Degrees.of(Constants.intakeAngleUp));
    }

    public void setPivot(int pov){
        if(pov == 0) pivotMotor.setVoltage(3);
        else if(pov == 180){
            pivotMotor.setVoltage(-6);
        }
        else if(pov == 90){
            indexerSystem.setVoltage(Volts.of(-10));
        }
        else{
            pivotMotor.set(0);
        }
    }

    public Command runIntakeCommand(){
        return run(() -> intakeRollers.setVoltage(-6));
    }

    public Command stopIntakeCommand(){
        return runOnce(() -> intakeRollers.setVoltage(0));
    }

    public Command raisePivot(){
        return run(() -> pivotMotor.setVoltage(2.4));
    }

    public Command lowerPivot(){
        return run(() -> pivotMotor.setVoltage(-2.4));
    }

    public static Intake getInstance(){
        if(intake == null) intake = new Intake();
        return intake;
    }
}
