package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.automations.Vision;
import frc.robot.test.ShooterTest;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorInterface extends SubsystemBase{
    private static OperatorInterface oi = null;
    private XboxController controller1;
    private Drive drivetrain = Drive.getInstance();
    private Vision vision = Vision.getInstance();
    private Intake ballIntake = Intake.getInstance();
    private Shooter ballShooter = Shooter.getInstance();
    private Climb climber = Climb.getInstance();
    private Telemetry telemetry = Telemetry.getInstance();
    private ShooterTest shooterTest = ShooterTest.getInstance();

    private OperatorInterface(){
        controller1 = new XboxController(0);
    }

    private void updateDrive(){
        drivetrain.driveInputHandler(-controller1.getRawAxis(1), -controller1.getRawAxis(0), -controller1.getRawAxis(4), controller1.getYButtonPressed(), controller1.getAButtonPressed());
        drivetrain.updatePoseEstimator();
    }

    private void updateClimb(){
        climber.climbInputHandler(controller1.getPOV());
    }

    private void updateIntake(){
        ballIntake.intakeInputHandler(controller1.getLeftBumperButtonPressed(), controller1.getLeftTriggerAxis());
    }

    private void updateShooter(){
        ballShooter.shooterInputManager(controller1.getRightBumperButtonPressed(), controller1.getRightTriggerAxis());
        // shooterTest.SetTestVelocity(controller1.getBButtonPressed());
        shooterTest.runFlyWheelSysID(controller1.getBButtonPressed());
    }

    private void updateTelemetry(){
        telemetry.update();
    }

    private void updateVision(){
        vision.updateSimVision(drivetrain.getRobotPose());
    }
    
    @Override
    public void periodic(){
        updateDrive();
        updateVision();
        updateTelemetry();
        updateIntake();
        updateShooter();
        updateClimb();
    }

    public static OperatorInterface getInstance(){
        if (oi == null){
            oi = new OperatorInterface();
        }
        return oi;
    }
}