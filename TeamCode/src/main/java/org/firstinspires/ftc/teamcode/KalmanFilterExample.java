package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Explicitly declare the KalmanFilter class
class KalmanFilter {
    private double[] state = {0.0, 0.0, 0.0}; // X, Y, Theta
    private double[][] covariance = {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
    };

    public void predict(double deltaX, double deltaY, double deltaTheta, double[][] processNoise) {
        // Update state prediction
        state[0] += deltaX;
        state[1] += deltaY;
        state[2] += deltaTheta;

        // Update covariance with process noise
        for (int i = 0; i < covariance.length; i++) {
            for (int j = 0; j < covariance[i].length; j++) {
                covariance[i][j] += processNoise[i][j];
            }
        }
    }

    public void update(double[] measurement, double[][] measurementNoise) {
        // Simplified Kalman gain calculation (identity matrix assumption)
        double[][] kalmanGain = new double[covariance.length][covariance[0].length];
        for (int i = 0; i < kalmanGain.length; i++) {
            for (int j = 0; j < kalmanGain[i].length; j++) {
                kalmanGain[i][j] = covariance[i][j] / (covariance[i][j] + measurementNoise[i][j]);
            }
        }

        // Update state with measurement
        for (int i = 0; i < state.length; i++) {
            state[i] += kalmanGain[i][i] * (measurement[i] - state[i]);
        }

        // Update covariance
        for (int i = 0; i < covariance.length; i++) {
            for (int j = 0; j < covariance[i].length; j++) {
                covariance[i][j] = (1 - kalmanGain[i][j]) * covariance[i][j];
            }
        }
    }

    public double[] getState() {
        return state;
    }
}

// Main class
public class KalmanFilterExample extends LinearOpMode {
    private KalmanFilter kalmanFilter;
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        kalmanFilter = new KalmanFilter();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example motion
            moveToPosition(12, 0, 0); // Move forward 1'
            moveToPosition(0, 24, 0); // Move right 2'
            moveToPosition(36, 0, 0); // Move forward 3'
            moveToPosition(0, 12, 0); // Move right 1'
            moveToPosition(-36, 0, 0); // Move back 3'
        }
    }

    private void moveToPosition(double forwardInches, double strafeInches, double rotationDegrees) {
        // Use encoder ticks and odometry to measure motion
        double deltaX = forwardInches; // Example from odometry
        double deltaY = strafeInches;
        double deltaTheta = rotationDegrees; // Example from IMU

        // Predict the state using the Kalman filter
        double[][] processNoise = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.05}};
        kalmanFilter.predict(deltaX, deltaY, deltaTheta, processNoise);

        // Simulate sensor measurements
        double[] measurement = {deltaX, deltaY, deltaTheta};
        double[][] measurementNoise = {{0.2, 0, 0}, {0, 0.2, 0}, {0, 0, 0.1}};
        kalmanFilter.update(measurement, measurementNoise);

        // Get the filtered state
        double[] state = kalmanFilter.getState();
        telemetry.addData("Position", "X: %.2f, Y: %.2f, Theta: %.2f", state[0], state[1], state[2]);
        telemetry.update();
    }
}
