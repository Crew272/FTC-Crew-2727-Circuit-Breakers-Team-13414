package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Drive", group = "Linear Opmode")
public class MecanumDrive1 extends LinearOpMode {

    private RobotHardware robot;

    // Constants for smooth acceleration and deceleration
    private static final double ACCELERATION_INCREMENT = 0.05;
    private static final double DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Variable to hold the previous motor power
        double previousPowerFL = 0, previousPowerFR = 0, previousPowerBL = 0, previousPowerBR = 0;

        while (opModeIsActive()) {
            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x * 1.1; // Strafing (adjusted for mecanum wheel skew)
            double rx = gamepad1.right_stick_x; // Rotation

            // Calculate motor powers
            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double rearLeftPower = y - x + rx;
            double rearRightPower = y + x - rx;

            // Normalize motor powers to keep within -1 to 1
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                rearLeftPower /= maxPower;
                rearRightPower /= maxPower;
            }

            // Smooth acceleration and deceleration
            frontLeftPower = smoothPower(previousPowerFL, frontLeftPower);
            frontRightPower = smoothPower(previousPowerFR, frontRightPower);
            rearLeftPower = smoothPower(previousPowerBL, rearLeftPower);
            rearRightPower = smoothPower(previousPowerBR, rearRightPower);

            // Set motor powers
            robot.frontLeftDrive.setPower(frontLeftPower);
            robot.frontRightDrive.setPower(frontRightPower);
            robot.rearLeftDrive.setPower(rearLeftPower);
            robot.rearRightDrive.setPower(rearRightPower);

            // Update previous motor power variables
            previousPowerFL = frontLeftPower;
            previousPowerFR = frontRightPower;
            previousPowerBL = rearLeftPower;
            previousPowerBR = rearRightPower;

            // Telemetry for debugging
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("RR Power", rearRightPower);
            telemetry.update();
        }
    }

    // Smooth power adjustment
    private double smoothPower(double previousPower, double targetPower) {
        if (Math.abs(targetPower - previousPower) > ACCELERATION_INCREMENT) {
            if (targetPower > previousPower) {
                return previousPower + ACCELERATION_INCREMENT;
            } else {
                return previousPower - ACCELERATION_INCREMENT;
            }
        } else {
            return targetPower;
        }
    }
}
