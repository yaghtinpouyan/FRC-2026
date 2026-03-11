package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.idConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Indexer extends SubsystemBase {
    private static Indexer indexer = null;

    private TalonFX indexerMotor;

    private final SmartMotorControllerConfig falconConfig;
    private SmartMotorController indexerSystem;

    private Indexer(){
        indexerMotor = new TalonFX(idConstants.falcon500_I1);

        falconConfig = new SmartMotorControllerConfig(this)
                            .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
                            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                            .withIdleMode(MotorMode.COAST)
                            .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
                            .withStatorCurrentLimit(Amps.of(20))
                            .withMotorInverted(true)
                            .withClosedLoopRampRate(Seconds.of(0.25))
                            .withOpenLoopRampRate(Seconds.of(0.25))
                            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withControlMode(ControlMode.CLOSED_LOOP);

        indexerSystem = new TalonFXWrapper(indexerMotor, DCMotor.getFalcon500(1), falconConfig);
    }

    //Hopper Code
    public void runHopper(boolean indexF){
        //Account for movement vector later to save power
        if(indexF) indexerSystem.setVoltage(Volts.of(3));
        else{
            if(indexF) indexerSystem.setVoltage(Volts.of(-3));
        }
    } 

    public static Indexer getInstance(){
        if(indexer == null) indexer = new Indexer();
        return indexer;
    }
}
