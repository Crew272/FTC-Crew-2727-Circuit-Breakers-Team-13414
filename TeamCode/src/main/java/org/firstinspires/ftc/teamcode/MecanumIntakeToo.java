package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Mecanum Drive with Reassigned Grabber Controls", group = "TeleOp")
public class MecanumIntakeToo extends LinearOpMode {

    private RobotHardware robot;
    private boolean upAndDownState = false; // Tracks toggle state for upAndDownServo
    private boolean previousBState = false; // Tracks the previous state of the B button
    private boolean claw2GrabState = false; // Tracks the toggle state for Claw2 Grab Servo
    private boolean previousAState = false; // Tracks the previous state of the A button
    private double arm1RotationPosition = 0.9; // Start at .9
    private double slideRotationPosition = 0.0; // Start at .9

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        // Initialize the servo position before the loop starts
        robot.slideRightServo.setPosition(slideRotationPosition);
        robot.arm1RotationServo.setPosition(arm1RotationPosition);
        robot.claw1GrabServo.setPosition(1.0); //close claw on specimen
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // **Driver Controls (Gamepad A)**
//            double drive = -gamepad1.left_stick_y; // Forward/Backward
//            double strafe = gamepad1.left_stick_x; // Left/Right
//            double rotate = gamepad1.right_stick_x; // Rotation
            double speedModifier = 0.5; // Reduce speed (1.0 = full speed, 0.3 = slow speed)

            double drive = -gamepad1.left_stick_y * speedModifier;
            double strafe = gamepad1.left_stick_x * speedModifier;
            double rotate = gamepad1.right_stick_x * speedModifier;

            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double rearLeftPower = drive - strafe + rotate;
            double rearRightPower = drive + strafe - rotate;

            double maxPower = Math.max(1.0, Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(rearLeftPower),
                                    Math.abs(rearRightPower)))));

            robot.frontLeftDrive.setPower(frontLeftPower / maxPower);
            robot.frontRightDrive.setPower(frontRightPower / maxPower);
            robot.rearLeftDrive.setPower(rearLeftPower / maxPower);
            robot.rearRightDrive.setPower(rearRightPower / maxPower);

            // **Operator Controls (Gamepad B)**

            // Grabber 1
            //robot.grabber1LiftMotor.setPower(-gamepad2.left_stick_y);
            double liftPower = gamepad2.left_stick_y; // Invert for correct movement
            // Reset encoder when the limit switch is pressed
            if (robot.isLiftAtBottom()) {
                robot.grabber1LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.grabber1LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // Prevent downward movement if limit switch is engaged
            if (robot.isLiftAtBottom() && liftPower > 0) {
                robot.grabber1LiftMotor.setPower(0);
            }
            // Prevent going too high
            else if (robot.grabber1LiftMotor.getCurrentPosition() <= RobotHardware.LIFT_MAX_HEIGHT_TICKS && liftPower < 0) {
                robot.grabber1LiftMotor.setPower(0);
            }
            // Allow movement within limits
            else {
                robot.grabber1LiftMotor.setPower(liftPower);
            }

            // Read joystick X-axis
            double armMove = gamepad2.left_stick_x;

            // Deadzone to prevent jitter
            if (Math.abs(armMove) > 0.05) {
                // Quadratic scaling for smooth acceleration
                double movement = Math.signum(armMove) * Math.pow(Math.abs(armMove), 2) * 0.01; // Adjust for speed

                // Update servo position gradually based on joystick direction
                arm1RotationPosition += movement;
            }

            // Clamp position to servo limits (0.0 to 1.0)
            arm1RotationPosition = Math.max(0.0, Math.min(0.9, arm1RotationPosition));

            // Apply new position to servo
            robot.arm1RotationServo.setPosition(arm1RotationPosition);



            if (gamepad2.left_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(0.0); // Open Claw1
            } else if (gamepad2.right_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(1.0); // Close Claw1
            }

            // Grabber 2
            // Read joystick Y-axis
            double slidePower = gamepad2.right_stick_y;

            // Deadzone to prevent jitter
            if (Math.abs(slidePower) > 0.05) {
                // Quadratic scaling for smooth acceleration
                double movement = Math.signum(slidePower) * Math.pow(Math.abs(slidePower), 2) * 0.01; // Adjust for speed

                // Update servo position gradually based on joystick direction
                slideRotationPosition += movement;
            }

            // Clamp position to servo limits (0.0 to 1.0)
            slideRotationPosition = Math.max(0.0, Math.min(1.0, arm1RotationPosition));

            // Apply new position to servo
            robot.slideRightServo.setPosition(slideRotationPosition);

            // double slidePower = gamepad2.right_stick_y;
            // robot.slideLeftServo.setPosition(0.5 + slidePower / 2); // Slide Left Servo
            // robot.slideRightServo.setPosition(0.5 - slidePower / 2); // Slide Right Servo

//            double claw2Rotation = gamepad2.right_stick_x;
//            if (claw2Rotation != 0) {
//                robot.claw2RotationServo.setPosition(0.5 + (claw2Rotation / 2)); // Adjust Claw2 Rotation
//            }

            // Claw2 Grab Toggle using A Button
            if (gamepad2.a && !previousAState) {
                claw2GrabState = !claw2GrabState;
                robot.claw2GrabServo.setPosition(claw2GrabState ? 1.0 : 0.0);
            }
            previousAState = gamepad2.a;

            // UpAndDownServo Toggle using B Button
            if (gamepad2.b && !previousBState) {
                upAndDownState = !upAndDownState;
                robot.upAndDownServo.setPosition(upAndDownState ? 1.0 : 0.0);
            }
            previousBState = gamepad2.b;

            // **NEW: Continuous Rotation Servo Control**
            if (gamepad2.left_bumper) {
                robot.intake1.setPower(-1.0); // Counterclockwise
            } else if (gamepad2.right_bumper) {
                robot.intake1.setPower(1.0);  // Clockwise
            } else {
                robot.intake1.setPower(0.0);  // Stop when no button is pressed
            }
            // Telemetry for Debugging
            telemetry.addData("Claw2 Grab State", claw2GrabState ? "Closed" : "Open");
            telemetry.addData("UpAndDownServo State", upAndDownState ? "Up" : "Down");
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("Arm Rotation Servo Position", arm1RotationPosition);
            // Telemetry for debugging
            telemetry.addData("Lift Position", robot.grabber1LiftMotor.getCurrentPosition());
            telemetry.addData("Limit Switch", robot.isLiftAtBottom() ? "Pressed" : "Not Pressed");
            telemetry.update();
        }
    }
}

