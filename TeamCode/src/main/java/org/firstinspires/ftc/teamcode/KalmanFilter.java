private KalmanFilter kalmanFilter;

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

