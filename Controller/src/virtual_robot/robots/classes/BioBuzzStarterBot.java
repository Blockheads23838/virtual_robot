package virtual_robot.robots.classes;

import javafx.fxml.FXML;


import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorControllerImpl;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;

@BotConfig(name = "BioBuzz Starter Bot", filename = "bio_buzz_starter_bot")
public class BioBuzzStarterBot extends VirtualBot {

    // Physical Kinematic Parameters
    private double wheelDiameter = 3.77953; // 96mm goBILDA wheels in inches
    private double trackWidth = 14.1732;    // Chassis track width in inches
    private double encoderTicksPerRotation = 537.7; // 312 RPM Yellow Jacket CPR
    private final double maxSpeedInchesPerSec = 60.0; // Max velocity for 312 RPM motors

    // Drive Motors (Differential / 6WD Skid-Steer)
    @FXML private DcMotorImpl leftDrive = null;
    @FXML private DcMotorImpl rightDrive = null;

    // Mechanism Motors
    private DcMotorImpl intake = null;
    private DcMotorExImpl launcher = null;

    // Continuous Rotation Servos
    private CRServoImpl windmillServo = null;
    private CRServoImpl leftIntakeServo = null;
    private CRServoImpl rightIntakeServo = null;

    public BioBuzzStarterBot() {
        super();
    }

    @Override
    protected void createHardwareMap() {
        hardwareMap = new HardwareMap();
      // 1. Create a motor controller with 4 ports for all DC motors
        DcMotorControllerImpl motorController = new DcMotorControllerImpl();

        // 2. Instantiate drive and mechanism motors on ports 0 through 3
        leftDrive = new DcMotorImpl(MotorType.Neverest40, motorController, 0);
        rightDrive = new DcMotorImpl(MotorType.Neverest40, motorController, 1);
        intake = new DcMotorImpl(MotorType.Neverest40, motorController, 2);
        launcher = new DcMotorExImpl(MotorType.Neverest40, motorController, 3);

        // 3. Instantiate CRServos passing max speed in degrees/sec (180.0)
        windmillServo = new CRServoImpl(180.0);
        leftIntakeServo = new CRServoImpl(180.0);
        rightIntakeServo = new CRServoImpl(180.0);

        // 4. Populate typed device mappings (required for hardwareMap.dcMotor and hardwareMap.get)
        hardwareMap.dcMotor.put("left_drive", leftDrive);
        hardwareMap.dcMotor.put("right_drive", rightDrive);
        hardwareMap.dcMotor.put("intake", intake);
        hardwareMap.dcMotor.put("launcher", launcher);

        hardwareMap.crservo.put("windmill", windmillServo);
        hardwareMap.crservo.put("left_intake_servo", leftIntakeServo);
        hardwareMap.crservo.put("right_intake_servo", rightIntakeServo);

        // 4. Map exact string keys in generic hardwareMap
        hardwareMap.put("left_drive", leftDrive);
        hardwareMap.put("right_drive", rightDrive);
        hardwareMap.put("intake", intake);
        hardwareMap.put("launcher", launcher);
        hardwareMap.put("windmill", windmillServo);
        hardwareMap.put("left_intake_servo", leftIntakeServo);
        hardwareMap.put("right_intake_servo", rightIntakeServo);
    }

    @Override
    public synchronized void updateStateAndSensors(double elapsedSec) {
        if (leftDrive != null) leftDrive.update(elapsedSec);
        if (rightDrive != null) rightDrive.update(elapsedSec);
        if (intake != null) intake.update(elapsedSec);
        if (launcher != null) launcher.update(elapsedSec);

        // Calculate power taking motor direction setting into account
        double leftPower = leftDrive != null ? leftDrive.getPower() * (leftDrive.getDirection() == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE ? -1 : 1) : 0.0;
        double rightPower = rightDrive != null ? rightDrive.getPower() * (rightDrive.getDirection() == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE ? -1 : 1) : 0.0;

        double leftSpeed = leftPower * maxSpeedInchesPerSec;
        double rightSpeed = rightPower * maxSpeedInchesPerSec;

        double forwardVelocity = (leftSpeed + rightSpeed) / 2.0;
        double angularVelocity = (rightSpeed - leftSpeed) / trackWidth;

        headingRadians += angularVelocity * elapsedSec;

        while (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        while (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;

        x += forwardVelocity * Math.cos(headingRadians) * elapsedSec;
        y += forwardVelocity * Math.sin(headingRadians) * elapsedSec;

        x = Math.max(-66.0, Math.min(66.0, x));
        y = Math.max(-66.0, Math.min(66.0, y));
    }

    @Override
    public synchronized void updateDisplay() {
        // Required: Updates the JavaFX screen pixels from x, y, and headingRadians
        super.updateDisplay();
    }

    @Override
    public synchronized void powerDownAndReset() {
        // Reset motors and zero servo power on OpMode stop
        leftDrive.stopAndReset();
        rightDrive.stopAndReset();
        intake.stopAndReset();
        launcher.stopAndReset();
        windmillServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
}
