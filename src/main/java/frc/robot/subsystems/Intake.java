package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        indexer.setVoltage(8);
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

    public void setPivot(int pov){
        if(pov == 0) pivot.setVoltage(2.4);
        else if(pov == 180){
            pivot.setVoltage(-2.4);
        }
        else if(pov == 90){
            indexer.setVoltage(-8);
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
