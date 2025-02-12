package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotHardware1 {
    // Drivetrain Motors (Mecanum Wheels)
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor rearLeftDrive;
    public DcMotor rearRightDrive;

    // Odometry Wheels
    public DcMotor centerOdometry;
    public DcMotor rightOdometry;
    public DcMotor leftOdometry;

    // Arm and Slide Motors
    public DcMotor armMotor;
    public DcMotor slideMotor;

    // Intake Motor
    public DcMotor intakeMotor;

    // Servos
    public Servo grabberServo;
    public Servo intakePositionServo;

    // Touch Sensors (Limit Switches)
    public TouchSensor armZeroSwitch;
    public TouchSensor slideZeroSwitch;

    // Hardware Map
    private HardwareMap hardwareMap;

    // Constructor
    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initializeHardware();
    }

    // Initialize all hardware components
    private void initializeHardware() {
        // Drivetrain Motors
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        rearLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        rearRightDrive = hardwareMap.dcMotor.get("rearRightDrive");

        // Set motor directions (adjust as needed)
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor run modes for encoders
        setDrivetrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Odometry Wheels
        centerOdometry = hardwareMap.dcMotor.get("CenterOdometry");
        rightOdometry = hardwareMap.dcMotor.get("RightOdometry");
        leftOdometry = hardwareMap.dcMotor.get("LeftOdometry");

        // Configure odometry wheels for tracking
        centerOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centerOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Arm and Slide Motors
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        slideMotor = hardwareMap.dcMotor.get("slide_motor");

        // Configure arm and slide motors
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Intake Motor
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");

        // Servos
        grabberServo = hardwareMap.servo.get("grabber_servo");
        intakePositionServo = hardwareMap.servo.get("intake_position_servo");

        // Touch Sensors (Limit Switches)
        armZeroSwitch = hardwareMap.touchSensor.get("arm_zero_switch");
        slideZeroSwitch = hardwareMap.touchSensor.get("slide_zero_switch");
    }

    // Method to set drivetrain motor run modes
    public void setDrivetrainRunMode(DcMotor.RunMode runMode) {
        frontLeftDrive.setMode(runMode);
        frontRightDrive.setMode(runMode);
        rearLeftDrive.setMode(runMode);
        rearRightDrive.setMode(runMode);
    }

    // Method to control intake
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    // Method to control grabber servo
    public void setGrabberPosition(double position) {
        grabberServo.setPosition(position);
    }

    // Method to control intake position servo
    public void setIntakePosition(double position) {
        intakePositionServo.setPosition(position);
    }

    // Method to check if arm is at zero position
    public boolean isArmAtZero() {
        return armZeroSwitch.isPressed();
    }

    // Method to check if slide is at zero position
    public boolean isSlideAtZero() {
        return slideZeroSwitch.isPressed();
    }
}
