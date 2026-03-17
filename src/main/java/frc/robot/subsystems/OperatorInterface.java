package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorInterface extends SubsystemBase{
    private static OperatorInterface oi = null;
    private XboxController controller1;
    private XboxController controller2;
    private Drive drivetrain = Drive.getInstance();
    // private Vision vision = Vision.getInstance();
    private Intake ballIntake = Intake.getInstance();
    private Shooter ballShooter = Shooter.getInstance();
    // private Climb climber = Climb.getInstance();
    private Telemetry telemetry = Telemetry.getInstance();
    //private ShooterTest shooterTest = ShooterTest.getInstance();

    private OperatorInterface(){
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);
    }

    private void updateDrive(){
        //drivetrain.driveCommand(-controller1.getRawAxis(1), -controller1.getRawAxis(0), -controller1.getRawAx
        drivetrain.driveCommand(() -> -controller1.getRawAxis(1), 
            () -> -controller1.getRawAxis(0), () -> -controller1.getRawAxis(4));
    }

    private void updateIntake(){
        ballIntake.runRollers(controller2.getRawAxis(3), controller2.getRawButton(6));
        ballIntake.setPivot(controller2.getPOV());
        //ballIntake.unJam(controller2.getYButton());
    }

    private void updateShooter(){
        ballShooter.shooterInputManager(controller2.getRawAxis(2), controller2.getRawButton(5));

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
    }

    public static OperatorInterface getInstance(){
        if (oi == null){
            oi = new OperatorInterface();
        }
        return oi;
    }
}