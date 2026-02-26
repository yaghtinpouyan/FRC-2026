package frc.robot.test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeTest extends SubsystemBase {
    private static IntakeTest intakeTest = null;
    private intake ballIntake = intake.getInstance();
    double testIntakePower = SmartDashboard.getNumber("Intake Power", 0);

    public void setTestIntake(boolean runTest) {
        testIntakePower = SmartDashboard.getNumber("Intake Power", 0);

        if (runTest) {
            ballIntake.setIntakePivot();
            if (testIntakePower >= 0) ballIntake.runIntake(true);
            else ballIntake.runIntake(false);
            ballIntake.runHopper(true);
        } else {
            ballIntake.runIntake(false);
            ballIntake.runHopper(false);
        }
    }

    public static IntakeTest getInstance() {
        if (intakeTest == null) {
            intakeTest = new IntakeTest();
        }
        return intakeTest;
    }
}