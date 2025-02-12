package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotHardware {
    // Drivetrain Motors (Mecanum Wheels)
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor rearLeftDrive;
    public DcMotor rearRightDrive;

    // Grabber 1 Components
    public DcMotor grabber1LiftMotor;
    public Servo arm1RotationServo;
    public Servo claw1GrabServo;

    // Grabber 2 Components
    public Servo slideLeftServo;
    public Servo slideRightServo;
    public Servo claw2RotationServo;
    public Servo claw2GrabServo;
    public Servo upAndDownServo; // Added servo for up and down movement

    // Hardware Map
    private HardwareMap hardwareMap;

    // Constructor
    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initializeHardware();
    }

    private void initializeHardware() {
        // Drivetrain Motors Initialization
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        rearLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        rearRightDrive = hardwareMap.dcMotor.get("rearRightDrive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Grabber 1 Initialization
        grabber1LiftMotor = hardwareMap.dcMotor.get("grabber1LiftMotor");
        arm1RotationServo = hardwareMap.servo.get("arm1RotationServo");
        claw1GrabServo = hardwareMap.servo.get("claw1GrabServo");

        // Grabber 2 Initialization
        slideLeftServo = hardwareMap.servo.get("slideLeftServo");
        slideRightServo = hardwareMap.servo.get("slideRightServo");
        claw2RotationServo = hardwareMap.servo.get("claw2RotationServo");
        claw2GrabServo = hardwareMap.servo.get("claw2GrabServo");
        upAndDownServo = hardwareMap.servo.get("upAndDownServo"); // New servo
    }
}
