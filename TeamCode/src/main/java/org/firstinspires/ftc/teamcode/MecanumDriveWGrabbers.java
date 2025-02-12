package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Mecanum Drive with Grabbers", group = "TeleOp")
public class MecanumDriveWGrabbers extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Driver Controls (Gamepad A)
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
                            Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower)))));

            robot.frontLeftDrive.setPower(frontLeftPower / maxPower);
            robot.frontRightDrive.setPower(frontRightPower / maxPower);
            robot.rearLeftDrive.setPower(rearLeftPower / maxPower);
            robot.rearRightDrive.setPower(rearRightPower / maxPower);

            // Operator Controls (Gamepad B)

            // Grabber 1 (Left Side)
            robot.grabber1LiftMotor.setPower(-gamepad2.left_stick_y);

            if (gamepad2.left_bumper) {
                robot.arm1RotationServo.setPosition(0.0); // Rotate Left
            } else if (gamepad2.right_bumper) {
                robot.arm1RotationServo.setPosition(1.0); // Rotate Right
            }

            if (gamepad2.left_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(0.0); // Open Claw
            } else if (gamepad2.right_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(1.0); // Close Claw
            }

            // Grabber 2 (Right Side)
            double slidePower = gamepad2.right_stick_y;
            robot.slideLeftServo.setPosition(0.5 + slidePower / 2);
            robot.slideRightServo.setPosition(0.5 - slidePower / 2);

            if (gamepad2.a) {
                robot.claw2RotationServo.setPosition(0.0); // Rotate Claw Left
            } else if (gamepad2.b) {
                robot.claw2RotationServo.setPosition(1.0); // Rotate Claw Right
            }

            if (gamepad2.left_trigger > 0.5) {
                robot.claw2GrabServo.setPosition(0.0); // Open Claw
            } else if (gamepad2.right_trigger > 0.5) {
                robot.claw2GrabServo.setPosition(1.0); // Close Claw

                telemetry.update();
            }
        }
    }
}
