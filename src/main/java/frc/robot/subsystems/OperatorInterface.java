package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.automations.Vision;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorInterface extends SubsystemBase{
    private static OperatorInterface oi = null;
    private XboxController controller1;
    private XboxController controller2;
    private Drive drivetrain = Drive.getInstance();
    private Vision vision = Vision.getInstance();
    private Intake ballIntake = Intake.getInstance();
    private Shooter ballShooter = Shooter.getInstance();
    private Telemetry telemetry = Telemetry.getInstance();

    private OperatorInterface(){
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);
    }

    private void updateDrive(){
        drivetrain.driveInputHandler(() -> -controller1.getRawAxis(1), 
            () -> -controller1.getRawAxis(0), () -> -controller1.getRawAxis(4), controller1.getRightBumperButton());
    }

    private void updateIntake(){
        ballIntake.runRollers(controller2.getRawAxis(3), controller2.getRawButton(6));
        ballIntake.setPivot(controller2.getPOV());
    }

    private void updateShooter(){
        // ballShooter.shooterInputManager(
        //     controller2.getLeftTriggerAxis(), 
        //     controller2.getLeftBumperButton(), 
        //     controller2.getYButtonPressed(), 
        //     controller2.getAButtonPressed(), 
        //     controller2.getBButton()
        // );
        ballShooter.runHoodSysID(controller2.getBButtonPressed());
    }

    private void updateVision(){
        vision.updateVision();
    }

    private void updateTelemetry(){
        telemetry.update();
    }
    
    @Override
    public void periodic(){
        //updateDrive();
        //updateTelemetry();
        //updateIntake();
        updateShooter();
        //updateVision();
    }

    public static OperatorInterface getInstance(){
        if (oi == null){
            oi = new OperatorInterface();
        }
        return oi;
    }
}