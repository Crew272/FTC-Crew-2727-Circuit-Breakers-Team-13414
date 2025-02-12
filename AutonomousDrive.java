package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;

@Autonomous(name = "Autonomous Drive with Pinpoint Odometry", group = "Autonomous")
public class AutonomousDrive extends LinearOpMode {

    private RobotHardware robot;
    private I2cDeviceSynch odometryDevice;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        // Initialize the odometry device
        odometryDevice = hardwareMap.get(I2cDeviceSynch.class, "odometryDevice");
        odometryDevice.setI2cAddress(I2cAddr.create8bit(0x3C)); // Example I2C address, adjust as needed
        odometryDevice.engage();

        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        // Autonomous movements
        driveForward(12); // Drive forward 1 foot
        strafeRight(36);  // Strafe right 3 feet
        driveForward(24); // Drive forward 2 feet
        driveBackward(36); // Drive backward 3 feet
        driveForward(36); // Drive forward 3 feet
        strafeRight(6);   // Strafe right 6 inches
        driveForward(36); // Drive forward 3 feet

        telemetry.addLine("Autonomous complete!");
        telemetry.update();
    }

    private void driveForward(double inches) {
        moveRobot(inches, inches, inches, inches);
    }

    private void driveBackward(double inches) {
        moveRobot(-inches, -inches, -inches, -inches);
    }

    private void strafeRight(double inches) {
        moveRobot(inches, -inches, -inches, inches);
    }

    private void moveRobot(double flInches, double frInches, double rlInches, double rrInches) {
        int flTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (flInches * TICKS_PER_INCH);
        int frTarget = robot.frontRightDrive.getCurrentPosition() + (int) (frInches * TICKS_PER_INCH);
        int rlTarget = robot.rearLeftDrive.getCurrentPosition() + (int) (rlInches * TICKS_PER_INCH);
        int rrTarget = robot.rearRightDrive.getCurrentPosition() + (int) (rrInches * TICKS_PER_INCH);

        robot.frontLeftDrive.setTargetPosition(flTarget);
        robot.frontRightDrive.setTargetPosition(frTarget);
        robot.rearLeftDrive.setTargetPosition(rlTarget);
        robot.rearRightDrive.setTargetPosition(rrTarget);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(0.5);
        robot.frontRightDrive.setPower(0.5);
        robot.rearLeftDrive.setPower(0.5);
        robot.rearRightDrive.setPower(0.5);

        while (opModeIsActive() && robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy()
                && robot.rearLeftDrive.isBusy() && robot.rearRightDrive.isBusy()) {
            telemetry.addData("FL Target", flTarget);
            telemetry.addData("FL Position", robot.frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR Target", frTarget);
            telemetry.addData("FR Position", robot.frontRightDrive.getCurrentPosition());
            telemetry.addData("RL Target", rlTarget);
            telemetry.addData("RL Position", robot.rearLeftDrive.getCurrentPosition());
            telemetry.addData("RR Target", rrTarget);
            telemetry.addData("RR Position", robot.rearRightDrive.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
        setRunModeUsingEncoders();
    }

    private void stopMotors() {
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
    }

    private void setRunModeUsingEncoders() {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
