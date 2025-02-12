package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous ProgramK", group = "Linear Opmode")
public class AutonomousProgramK extends LinearOpMode {

    private RobotHardware robot;
    private KalmanFilter kalmanFilter;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        kalmanFilter = new KalmanFilter();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward 1'
            moveToPosition(12, 0, 0); // Distance in Inches (Forward, Strafe, Rotate)

            // Move right 2'
            moveToPosition(0, 24, 0);

            // Move forward 3'
            moveToPosition(36, 0, 0);

            // Move right 1'
            moveToPosition(0, 12, 0);

            // Move back 3'
            moveToPosition(-36, 0, 0);
        }
    }

    private void moveToPosition(double forwardInches, double strafeInches, double rotationDegrees) {
        final double TICKS_PER_INCH = 1440 / (4 * Math.PI); // Example for Gobilda wheel encoder
        final double TICKS_PER_DEGREE = 15; // Example rotation adjustment

        int forwardTarget = (int) (forwardInches * TICKS_PER_INCH);
        int strafeTarget = (int) (strafeInches * TICKS_PER_INCH);
        int rotationTarget = (int) (rotationDegrees * TICKS_PER_DEGREE);

        int leftFrontTarget = forwardTarget + strafeTarget + rotationTarget;
        int rightFrontTarget = forwardTarget - strafeTarget - rotationTarget;
        int leftRearTarget = forwardTarget - strafeTarget + rotationTarget;
        int rightRearTarget = forwardTarget + strafeTarget - rotationTarget;

        // Predict the position using the Kalman filter
        double deltaX = forwardInches;
        double deltaY = strafeInches;
        double deltaTheta = rotationDegrees;
        double[][] processNoise = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.05}};
        kalmanFilter.predict(deltaX, deltaY, deltaTheta, processNoise);

        // Simulate measurements from odometry and IMU
        double[] measurement = {
                robot.leftOdometry.getCurrentPosition() / TICKS_PER_INCH,
                robot.centerOdometry.getCurrentPosition() / TICKS_PER_INCH,
                robot.rightOdometry.getCurrentPosition() / TICKS_PER_INCH
        };
        double[][] measurementNoise = {{0.2, 0, 0}, {0, 0.2, 0}, {0, 0, 0.1}};
        kalmanFilter.update(measurement, measurementNoise);

        // Get the filtered state
        double[] state = kalmanFilter.getState();
        telemetry.addData("Filtered Position", "X: %.2f, Y: %.2f, Theta: %.2f", state[0], state[1], state[2]);
        telemetry.update();

        // Reset odometry encoders
        robot.leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.centerOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftOdometry.setTargetPosition(leftFrontTarget);
        robot.rightOdometry.setTargetPosition(rightFrontTarget);
        robot.centerOdometry.setTargetPosition(leftRearTarget);

        robot.leftOdometry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightOdometry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.centerOdometry.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(0.5);
        robot.frontRightDrive.setPower(0.5);
        robot.rearLeftDrive.setPower(0.5);
        robot.rearRightDrive.setPower(0.5);

        while (opModeIsActive() &&
                (robot.leftOdometry.isBusy() || robot.rightOdometry.isBusy() || robot.centerOdometry.isBusy())) {
            telemetry.addData("Odometry Targets", "LF: %d, RF: %d, LR: %d", leftFrontTarget, rightFrontTarget, leftRearTarget);
            telemetry.addData("Positions", "LF: %d, RF: %d, LR: %d",
                    robot.leftOdometry.getCurrentPosition(),
                    robot.rightOdometry.getCurrentPosition(),
                    robot.centerOdometry.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);

        robot.leftOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.centerOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

