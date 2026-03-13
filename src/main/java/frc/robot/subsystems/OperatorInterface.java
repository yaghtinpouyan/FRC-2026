package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Intake;

public class OperatorInterface extends SubsystemBase{
    private static OperatorInterface oi = null;
    private XboxController controller1;
    private Drive drivetrain = Drive.getInstance();
    // private Vision vision = Vision.getInstance();
    private Intake ballIntake = Intake.getInstance();
    //private Shooter ballShooter = Shooter.getInstance();
    // private Climb climber = Climb.getInstance();
    private Telemetry telemetry = Telemetry.getInstance();
    //private ShooterTest shooterTest = ShooterTest.getInstance();

    private OperatorInterface(){
        controller1 = new XboxController(0);
    }

    private void updateDrive(){
        //drivetrain.driveCommand(-controller1.getRawAxis(1), -controller1.getRawAxis(0), -controller1.getRawAx
        drivetrain.driveCommand(() -> -controller1.getRawAxis(0), 
            () -> -controller1.getRawAxis(1), () -> -controller1.getRawAxis(4));
    }

    // private void updateClimb(){
    //     climber.climbInputHandler(controller1.getPOV());
    // }

    private void updateIntake(){
        ballIntake.runRollers(controller1.getRightTriggerAxis());
        ballIntake.setPivot(controller1.getLeftBumperButton(), controller1.getRightBumperButton());
    }

    // private void updateShooter(){
    //     ballShooter.shooterInputManager(controller1.getRawAxis(3));
    // }

    private void updateTelemetry(){
        telemetry.update();
    }

    // private void updateVision(){
    //     vision.updateSimVision(drivetrain.getPose());
    // }
    
    @Override
    public void periodic(){
        updateDrive();
        updateTelemetry();
        //updateIntake();
        //updateShooter();
    }

    public static OperatorInterface getInstance(){
        if (oi == null){
            oi = new OperatorInterface();
        }
        return oi;
    }
}