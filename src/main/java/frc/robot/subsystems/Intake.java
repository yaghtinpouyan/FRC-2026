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
        intakeRollers = new TalonFX(idConstants.falcon500_I2);
        indexer = new TalonFX(idConstants.falcon500_I2);
        pivot = new SparkMax(idConstants.neo_I3, MotorType.kBrushless);
    }

    public void runIndexer(){
        indexer.set(.5);
    }

    public void stopIndexer(){
        indexer.set(0);
    }

    public void runRollers(double input1){
        if(input1 > 0.1) pivot.set(.3);
    }

    public void setPivot(boolean input1, boolean input2){
        if(input1) pivot.set(.3);
        else if(input2){
            pivot.set(-0.3);
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
