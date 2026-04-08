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
        ballIntake.runRollers(controller1.getLeftTriggerAxis(), controller1.getLeftBumperButton());
        ballIntake.setPivotAngle(controller1.getPOV());
    }

    private void updateShooter(){
        ballShooter.shooterInputManager(
            controller1.getRightTriggerAxis(), 
            controller1.getLeftBumperButton(), 
            controller1.getYButtonPressed(), 
            controller1.getAButtonPressed(), 
            controller1.getBButton()
        );
       ballShooter.hoodAdjust();
    }

    private void updateVision(){
        vision.updateVision();
    }

    private void updateTelemetry(){
        telemetry.update();
    }
    
    @Override
    public void periodic(){
        updateDrive();
        updateTelemetry();
        updateIntake();
        updateShooter();
        updateVision();
    }

    public static OperatorInterface getInstance(){
        if (oi == null){
            oi = new OperatorInterface();
        }
        return oi;
    }
}