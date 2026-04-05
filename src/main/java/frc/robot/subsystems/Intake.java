package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.idConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase{
    private static Intake intake = null;

    TalonFX intakeRollers;
    TalonFX indexer;
    SparkMax pivotMotor;
    RelativeEncoder pivotEncoder;
    SparkClosedLoopController pivotController;
    private final SmartMotorControllerConfig falconConfig;
    private SmartMotorController indexerSystem;

    public boolean isIntaking = false;
    
    private Intake(){
        intakeRollers = new TalonFX(idConstants.falcon500_I1);
        indexer = new TalonFX(idConstants.falcon500_I2);
    
        falconConfig = new SmartMotorControllerConfig(this)
                            .withClosedLoopController(0.0001, 0, 0, RPM.of(6000), RotationsPerSecondPerSecond.of(100))
                            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 1)))
                            .withIdleMode(MotorMode.COAST)
                            .withStatorCurrentLimit(Amps.of(120))
                            .withSupplyCurrentLimit(Amps.of(60))
                            .withMotorInverted(true)
                            .withClosedLoopRampRate(Seconds.of(0.25))
                            .withOpenLoopRampRate(Seconds.of(0.25))
                            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                            .withControlMode(ControlMode.CLOSED_LOOP);

        // Arm Mechanism
        indexerSystem = new TalonFXWrapper(indexer, DCMotor.getFalcon500(1), falconConfig);

        //pivot
        pivotMotor = new SparkMax(idConstants.neo_I3, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.encoder.positionConversionFactor(360/47.53)
        .velocityConversionFactor(360/47.53/60.0);

        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.08)
        .i(0)
        .d(0.02)
        .feedForward.kV(0.02).kCos(0.79);
        
        pivotConfig.smartCurrentLimit(40);

        pivotConfig.closedLoop.maxMotion.cruiseVelocity(500)
        .maxAcceleration(1000)
        .allowedProfileError(0.5)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
        pivotConfig.inverted(true);
        pivotEncoder.setPosition(0);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIndexer(){
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

    private double calcPivotVolts(){
        if(Shooter.getInstance().isShooting){
            return 0.5;
        }
        else{
            return 3;
        }
    }

    private double calcTargetAngle(){
        if(Shooter.getInstance().isShooting){
            return 80.0;
        }
        else{
            return 0.0;
        }
    }

    public void setPivot(int pov){
        if(pov == 0){ 
            pivotMotor.setVoltage(calcPivotVolts());
        //pivotController.setSetpoint(0,ControlType.kMAXMotionPositionControl);
        }
        if(pov == 180){
            pivotMotor.setVoltage(-3);
            //pivotController.setSetpoint(115.5,ControlType.kMAXMotionPositionControl);
        }

        if(pov == -1 && (pivotEncoder.getPosition() >= 0 || pivotEncoder.getPosition() <= -120)){
            pivotMotor.setVoltage(0);
        }
        
        SmartDashboard.putNumber("Pivot Angle:", pivotEncoder.getPosition());
    }

    public void setPivotAngle(int pov){
        if(pov == 0) pivotController.setSetpoint(calcTargetAngle(),ControlType.kMAXMotionPositionControl);
        if(pov == 180){
            pivotController.setSetpoint(-120,ControlType.kMAXMotionPositionControl);
        }
        SmartDashboard.putNumber("Pivot Angle:", pivotEncoder.getPosition());
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
