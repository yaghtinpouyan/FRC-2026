package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.idConstants;

public class Intake extends SubsystemBase{
    private static Intake intake = null;

    TalonFX intakeRollers;
    TalonFX indexer;
    SparkMax pivot;
    
    private Intake(){
        intakeRollers = new TalonFX(idConstants.falcon500_I1);
        indexer = new TalonFX(idConstants.falcon500_I2);
        pivot = new SparkMax(idConstants.neo_I3, MotorType.kBrushless);
    }

    public void runIndexer(){
        indexer.setVoltage(12);
    }

    public void unJam(boolean input1){
        if(input1){
            indexer.setVoltage(-12);
        }
        else{
            indexer.setVoltage(0);
        }
    }

    public void stopIndexer(){
        indexer.setVoltage(0);
    }

    public void runRollers(double input1){
        if(input1 > 0.1) intakeRollers.setVoltage(10.5);
        else{
            intakeRollers.setVoltage(0);
        }
    }

    public void setPivot(boolean input1, boolean input2){
        if(input1) pivot.setVoltage(2.4);
        else if(input2){
            pivot.setVoltage(-2.4);
        }
        else{
            pivot.set(0);
        }
    }

    public static Intake getInstance(){
        if(intake == null) intake = new Intake();
        return intake;
    }
}
