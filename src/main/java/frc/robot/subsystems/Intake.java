package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.idConstants;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
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
    private final SmartMotorControllerConfig falconConfig;
    // private final SmartMotorControllerConfig neoConfig;
    private SmartMotorController indexerSystem;
    // private SmartMotorController pivotingSystem;
    // private ArmConfig pivotConfig;

    
    private Intake(){
        intakeRollers = new TalonFX(idConstants.falcon500_I1);
        indexer = new TalonFX(idConstants.falcon500_I2);
        pivotMotor = new SparkMax(idConstants.neo_I3, MotorType.kBrushless);
    
        falconConfig = new SmartMotorControllerConfig(this)
                            .withClosedLoopController(0.0001, 0, 0, RPM.of(6000), RotationsPerSecondPerSecond.of(100))
                            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 1)))
                            .withIdleMode(MotorMode.COAST)
                            .withStatorCurrentLimit(Amps.of(20))
                            .withMotorInverted(true)
                            .withClosedLoopRampRate(Seconds.of(0.25))
                            .withOpenLoopRampRate(Seconds.of(0.25))
                            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withControlMode(ControlMode.CLOSED_LOOP);

        //  neoConfig = new SmartMotorControllerConfig(this)
        //                 .withClosedLoopController(0.00016541, 0, 0, RPM.of(5500), RotationsPerSecondPerSecond.of(80))
        //                 .withGearing(new MechanismGearing(GearBox.fromReductionStages(47.32, 1)))
        //                 .withIdleMode(MotorMode.COAST)
        //                 .withStatorCurrentLimit(Amps.of(20))
        //                 .withMotorInverted(true)
        //                 .withClosedLoopRampRate(Seconds.of(0.25))
        //                 .withOpenLoopRampRate(Seconds.of(0.25))
        //                 .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        //                 .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        //                 .withControlMode(ControlMode.CLOSED_LOOP);

        indexerSystem = new TalonFXWrapper(indexer, DCMotor.getFalcon500(1), falconConfig);
        // pivotingSystem = new SparkWrapper(pivotMotor, DCMotor.getNEO(1), neoConfig);
        // pivotConfig = new ArmConfig(pivotingSystem)
        //                    .withLength(Meters.of(0.3366))
        //                    .withSoftLimits(Degrees.of(26.5), Degrees.of(152))
        //                    .withHardLimit(Degrees.of(26.5), Degrees.of(152))
        //                    .withTelemetry("Pivot", TelemetryVerbosity.HIGH)
        //                    .withMass(Kilograms.of(2.714))
        //                    .withStartingPosition(Degrees.of(0));
    }

    public void runIndexer(){
        AngularVelocity vel = RotationsPerSecond.of(6000);
        indexerSystem.setVelocity(vel);
    }

    public void stopIndexer(){
        indexer.setVoltage(0);
    }

    public void runRollers(double input1, boolean input2){
        if(input1 > 0.1) intakeRollers.setVoltage(-6);
        else if(input2){
            intakeRollers.setVoltage(6);
        }
        else{
            intakeRollers.setVoltage(0);
        }
    }

    // public void setPivotAngle(int pov){
    //     if(pov == 0) pivot.setPosition(Degrees.of(0));
    //     else if(pov == 180){
    //         pivotingSystem.setPosition(Degrees.of(123));
    //     }
    // }

    public void setPivot(int pov){
        if(pov == 0) pivotMotor.setVoltage(2.4);
        else if(pov == 180){
            pivotMotor.setVoltage(-2.4);
        }
        else if(pov == 90){
            indexer.setVoltage(-8);
        }
        else{
            pivotMotor.set(0);
        }
    }

    public static Intake getInstance(){
        if(intake == null) intake = new Intake();
        return intake;
    }
}
