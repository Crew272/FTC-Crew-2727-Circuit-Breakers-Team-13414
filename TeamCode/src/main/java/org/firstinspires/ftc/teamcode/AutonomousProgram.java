package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Program", group = "Linear Opmode")
public class AutonomousProgram extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward 1'
            drive(1.0, 0, 0, 12); // Power, Strafe, Rotation, Distance in Inches

            // Move right 2'
            drive(0, 1.0, 0, 24);

            // Move forward 3'
            drive(1.0, 0, 0, 36);

            // Move right 1'
            drive(0, 1.0, 0, 12);

            // Move back 3'
            drive(-1.0, 0, 0, 36);
        }
    }

    private void drive(double forward, double strafe, double rotate, double distanceInInches) {
        // Convert inches to encoder counts
        final double COUNTS_PER_REV = 537.6; // Example for NeveRest Orbital 20
        final double WHEEL_DIAMETER_INCHES = 4.0;
        final double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetPosition = (int) (distanceInInches * COUNTS_PER_INCH);

        // Set target positions for motors
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setTargetPosition((int) (targetPosition * (forward + strafe + rotate)));
        robot.frontRightDrive.setTargetPosition((int) (targetPosition * (forward - strafe - rotate)));
        robot.rearLeftDrive.setTargetPosition((int) (targetPosition * (forward - strafe + rotate)));
        robot.rearRightDrive.setTargetPosition((int) (targetPosition * (forward + strafe - rotate)));

        // Set to RUN_TO_POSITION mode
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers
        robot.frontLeftDrive.setPower(Math.abs(forward + strafe + rotate));
        robot.frontRightDrive.setPower(Math.abs(forward - strafe - rotate));
        robot.rearLeftDrive.setPower(Math.abs(forward - strafe + rotate));
        robot.rearRightDrive.setPower(Math.abs(forward + strafe - rotate));

        // Wait until target position is reached
        while (opModeIsActive() &&
                (robot.frontLeftDrive.isBusy() || robot.frontRightDrive.isBusy() ||
                        robot.rearLeftDrive.isBusy() || robot.rearRightDrive.isBusy())) {
            telemetry.addData("Path", "Driving to %d counts", targetPosition);
            telemetry.update();
        }

        // Stop all motion
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);

        // Reset to RUN_USING_ENCODER mode
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

