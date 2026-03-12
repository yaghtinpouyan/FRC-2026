package frc.robot.subsystems;

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
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase{
    private static Intake ballIntake = null;

    private TalonFX intakeMotor;
    private SparkMax pivotMotor;
    private Telemetry telemetry;

    private final SmartMotorControllerConfig falconConfig;
    private final SmartMotorControllerConfig neoConfig;
    private SmartMotorController pivotingSystem;
    private SmartMotorController intakingSystem;

    private LEDS leds = LEDS.getInstance();

    public boolean stateIntaking;

    private Intake(){
        intakeMotor = new TalonFX(idConstants.falcon500_I2);
        pivotMotor = new SparkMax(idConstants.neo_I3, MotorType.kBrushless);
        stateIntaking = false;

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

        neoConfig = new SmartMotorControllerConfig(this)
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

        pivotingSystem = new SparkWrapper(pivotMotor, DCMotor.getNEO(1), neoConfig);

        pivotConfig = new ArmConfig(pivotingSystem)
                           .withLength(Meters.of(0.3366))
                           .withSoftLimits(Degrees.of(26.5), Degrees.of(152))
                           .withHardLimit(Degrees.of(26.5), Degrees.of(152))
                           .withTelemetry("Pivot", TelemetryVerbosity.HIGH)
                           .withMass(Kilograms.of(2.714))
                           .withStartingPosition(Degrees.of(0));
                           
        intakingSystem = new TalonFXWrapper(intakeMotor, DCMotor.getFalcon500(1), falconConfig);
        
        pivot = new Arm(pivotConfig);
    }

    //Intaking Code
    
    //For scaling intake speed with drive speed
    public double calcIntakingVolts(){
        telemetry = Telemetry.getInstance();
        ChassisSpeeds chassisVel = telemetry.currentVelocity;
        double botVelocity = Math.hypot(chassisVel.vxMetersPerSecond, chassisVel.vyMetersPerSecond);
        double maxVelocity = Constants.maxDriveSpeed;
        double maxVolts = Constants.maxVolts;
        
        double scaling = 0.7 - (botVelocity / maxVelocity);
        return maxVolts*scaling;
    }

    //Pivot Code
    ArmConfig pivotConfig;

    
    private Arm pivot;

    public void setAngle(Angle angle){
        pivot.setAngle(angle);
    }

    //Increase the period of the cos function IMPORTANT
    public void rockingPivot(boolean reload){
        double pivotDir = getPivotDir();
        if(!reload){
            if(pivotDir == 1) setAngle(Degrees.of(Constants.rockingAngles[0]));
            else if(pivotDir == -1){
                setAngle(Degrees.of(Constants.rockingAngles[1]));
            }
        }
    }

    public void setIntakePivotUp(){
        setAngle(Degrees.of(Constants.intakeAngleUp));
    }

    public void setIntakePivotDown(){
        setAngle(Degrees.of(Constants.intakeAngleDown));
    }

    public double getPivotDir(){
        double k = (2*Math.PI)/Constants.osilationTIme;
        double dir = Math.cos(k*Timer.getFPGATimestamp());
        return dir;
    }

    public void sysId(double voltage,double step,double duration){
        pivot.sysId(Volts.of(voltage), Volts.of(step).per(Second), Seconds.of(duration)).schedule();
    }

    public void runIntake(boolean forward){
        if(forward) intakingSystem.setVoltage(Volts.of(3));
        else{
            intakingSystem.setVoltage(Volts.of(-3));
        }
    }

    public void changeIntakeState(boolean toggle){
        //To toggle between intaking and not intaking !!!! :thumbs_up:
        stateIntaking = toggle;
        if(stateIntaking) {
            setIntakePivotDown();
            intakingSystem.setVoltage(Volts.of(calcIntakingVolts()));
        }
        else{
            intakingSystem.setVoltage(Volts.of(0));
        }
    }

    public void intakeInputHandler(boolean raiseIntake, double input2){
        if (raiseIntake && !pivot.isNear(Degrees.of(Constants.intakeAngleUp), Degrees.of(5)).getAsBoolean()) {
            setIntakePivotUp();
        }
        if(input2 > 0.3){
            if (!pivot.isNear(Degrees.of(Constants.intakeAngleDown), Degrees.of(5)).getAsBoolean()){
                setIntakePivotDown();
            }
            leds.intakeSolid();

            if (pivot.isNear(Degrees.of(Constants.intakeAngleDown), Degrees.of(5)).getAsBoolean()){
                runIntake(false);
            }
        }
    }

    
    public static Intake getInstance(){
        if (ballIntake == null){
            ballIntake = new Intake();
        }
        return ballIntake;
    }
}
