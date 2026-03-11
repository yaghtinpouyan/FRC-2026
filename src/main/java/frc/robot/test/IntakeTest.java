package frc.robot.test;
import frc.robot.subsystems.Intake;
import frc.robot.constants.Constants;

public class IntakeTest {
    private static IntakeTest IntakeTest = null;
    private Intake ballIntake = Intake.getInstance();

    public void runIntakeSysID(){
        ballIntake.sysId(Constants.intakeSysIdMaxVoltage, Constants.intakeSysIdStep, Constants.intakeSysIdDuration);
    }

    public static IntakeTest getInstance(){
        if (IntakeTest == null){
            IntakeTest = new IntakeTest();
        }
        return IntakeTest;
    }
}
