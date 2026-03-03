package frc.robot.test;
import frc.robot.subsystems.intake;
import frc.robot.constants.Constants;

public class IntakeTest {
    private static IntakeTest IntakeTest = null;
    private intake ballIntake = intake.getInstance();

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
