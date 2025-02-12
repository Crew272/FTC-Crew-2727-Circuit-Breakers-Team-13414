package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Mecanum Drive with Reassigned Grabber Controls", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    private RobotHardware robot;
    private boolean upAndDownState = false; // Tracks toggle state for upAndDownServo
    private boolean previousBState = false; // Tracks the previous state of the B button
    private boolean claw2GrabState = false; // Tracks the toggle state for Claw2 Grab Servo
    private boolean previousAState = false; // Tracks the previous state of the A button

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // **Driver Controls (Gamepad A)**
            double drive = -gamepad1.left_stick_y; // Forward/Backward
            double strafe = gamepad1.left_stick_x; // Left/Right
            double rotate = gamepad1.right_stick_x; // Rotation

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
            robot.grabber1LiftMotor.setPower(-gamepad2.left_stick_y);

            if (gamepad2.left_bumper) {
                robot.arm1RotationServo.setPosition(0.0); // Rotate Arm1 Left
            } else if (gamepad2.right_bumper) {
                robot.arm1RotationServo.setPosition(1.0); // Rotate Arm1 Right
            }

            if (gamepad2.left_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(0.0); // Open Claw1
            } else if (gamepad2.right_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(1.0); // Close Claw1
            }

            // Grabber 2
            double slidePower = gamepad2.right_stick_y;
            robot.slideLeftServo.setPosition(0.5 + slidePower / 2); // Slide Left Servo
            robot.slideRightServo.setPosition(0.5 - slidePower / 2); // Slide Right Servo

            double claw2Rotation = gamepad2.right_stick_x;
            if (claw2Rotation != 0) {
                robot.claw2RotationServo.setPosition(0.5 + (claw2Rotation / 2)); // Adjust Claw2 Rotation
            }

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

            // Telemetry for Debugging
            telemetry.addData("Claw2 Grab State", claw2GrabState ? "Closed" : "Open");
            telemetry.addData("UpAndDownServo State", upAndDownState ? "Up" : "Down");
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("Claw2 Rotation", claw2Rotation);
            telemetry.update();
        }
    }
}
