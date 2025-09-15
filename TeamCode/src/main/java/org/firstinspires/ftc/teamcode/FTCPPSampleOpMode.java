/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * This OpMode demonstrates mecanum wheel drive control with goBilda Pinpoint odometry.
 * 
 * Motor Configuration:
 * - RightFront: Forward direction
 * - RightBack: Forward direction  
 * - LeftFront: Reverse direction
 * - LeftBack: Reverse direction
 * 
 * Odometry System:
 * - goBilda Pinpoint encoder for precise position tracking
 * - Real-time pose estimation (X, Y, Heading)
 * - Coordinate-based autonomous navigation
 * 
 * Speed is set to 0.1 (one tenth of maximum speed) for precise control.
 * 
 * The code provides coordinate-based movements using Pinpoint odometry:
 * - Precise position tracking
 * - Coordinate-based navigation
 * - Real-time pose feedback
 */

@Autonomous(name="FTCPP Sample OpMode", group="Custom")
public class FTCPPSampleOpMode extends LinearOpMode {

    /* Declare OpMode members */
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    
    // goBilda Pinpoint odometry sensor
    private GoBildaPinpointDriver pinpoint = null;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    // Motor speed constant - one tenth of maximum speed
    static final double DRIVE_SPEED = 0.1;
    
    // Position tracking using Pinpoint odometry
    private Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    
    // Movement constants for Pinpoint configuration
    static final double PINPOINT_X_OFFSET = -3.31;     // X pod offset in inches (left positive)
    static final double PINPOINT_Y_OFFSET = -6.61;     // Y pod offset in inches (forward positive)
    static final double POSITION_TOLERANCE = 0.5;      // Position tolerance in inches
    static final double HEADING_TOLERANCE = 2.0;       // Heading tolerance in degrees
    
    @Override
    public void runOpMode() {
        
        // Initialize the drive system variables
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        
        // Initialize Pinpoint odometry sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        
        // Set motor directions according to specifications
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        // Set motors to brake when zero power is applied
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Configure Pinpoint odometry sensor
        configurePinpoint();
        
        // Initialize position tracking with Pinpoint
        currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        pinpoint.setPosition(currentPose);
        
        // Send telemetry message to indicate successful initialization
        telemetry.addData("Status", "Initialized with Pinpoint Odometry");
        telemetry.addData("Drive Speed", "%.1f", DRIVE_SPEED);
        telemetry.addData("Starting Position", "(%.1f, %.1f)", 
                         currentPose.getX(DistanceUnit.INCH), 
                         currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Status", "Configured and Ready");
        telemetry.update();
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        
        // Run coordinate-based autonomous sequence
        if (opModeIsActive()) {
            
            // Coordinate-based autonomous sequence with Pinpoint precision
            telemetry.addData("Path", "Starting Pinpoint-guided autonomous sequence");
            telemetry.addData("Target Path", "(0,0) -> (20,20) -> (40,20)");
            telemetry.addData("Odometry System", "goBilda Pinpoint");
            telemetry.addData("Position Tolerance", "±%.1f inches", POSITION_TOLERANCE);
            telemetry.update();
            
            // Robot starts at position (0,0) - update Pinpoint position
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            telemetry.addData("Current Position", "(%.1f, %.1f)", 
                             currentPose.getX(DistanceUnit.INCH), 
                             currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Current Heading", "%.1f°", 
                             currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            sleep(1000);
            
            // Move to position (20,20) - Diagonal movement
            telemetry.addData("Waypoint 1", "Moving to (20,20)");
            telemetry.update();
            moveToPosition(20.0, 20.0);
            
            // Display waypoint reached information
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            telemetry.addData("Waypoint 1", "Reached (%.1f, %.1f)", 
                             currentPose.getX(DistanceUnit.INCH), 
                             currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            sleep(1000);
            
            // Move to position (40,20) - Lateral movement
            telemetry.addData("Waypoint 2", "Moving to (40,20)");
            telemetry.update();
            moveToPosition(40.0, 20.0);
            
            // Display final waypoint information
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            telemetry.addData("Waypoint 2", "Reached (%.1f, %.1f)", 
                             currentPose.getX(DistanceUnit.INCH), 
                             currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            
            // Stop all motors
            stopAllMotors();
            
            // Final position update
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            
            telemetry.addData("Path", "Complete");
            telemetry.addData("Final Position", "(%.1f, %.1f)", 
                             currentPose.getX(DistanceUnit.INCH), 
                             currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Final Heading", "%.1f°", 
                             currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Run Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
    }
    
    /**
     * Drive forward using mecanum wheels
     */
    public void driveForward(double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rightFront.setPower(DRIVE_SPEED);
            rightBack.setPower(DRIVE_SPEED);
            leftFront.setPower(DRIVE_SPEED);
            leftBack.setPower(DRIVE_SPEED);
            
            telemetry.addData("Action", "Driving Forward");
            telemetry.addData("Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
        stopAllMotors();
    }
    
    /**
     * Drive backward using mecanum wheels
     */
    public void driveBackward(double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rightFront.setPower(-DRIVE_SPEED);
            rightBack.setPower(-DRIVE_SPEED);
            leftFront.setPower(-DRIVE_SPEED);
            leftBack.setPower(-DRIVE_SPEED);
            
            telemetry.addData("Action", "Driving Backward");
            telemetry.addData("Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
        stopAllMotors();
    }
    
    /**
     * Strafe right using mecanum wheels
     */
    public void strafeRight(double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rightFront.setPower(-DRIVE_SPEED);
            rightBack.setPower(DRIVE_SPEED);
            leftFront.setPower(DRIVE_SPEED);
            leftBack.setPower(-DRIVE_SPEED);
            
            telemetry.addData("Action", "Strafing Right");
            telemetry.addData("Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
        stopAllMotors();
    }
    
    /**
     * Strafe left using mecanum wheels
     */
    public void strafeLeft(double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rightFront.setPower(DRIVE_SPEED);
            rightBack.setPower(-DRIVE_SPEED);
            leftFront.setPower(-DRIVE_SPEED);
            leftBack.setPower(DRIVE_SPEED);
            
            telemetry.addData("Action", "Strafing Left");
            telemetry.addData("Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
        stopAllMotors();
    }
    
    /**
     * Rotate left (counter-clockwise) using mecanum wheels
     */
    public void rotateLeft(double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rightFront.setPower(DRIVE_SPEED);
            rightBack.setPower(DRIVE_SPEED);
            leftFront.setPower(-DRIVE_SPEED);
            leftBack.setPower(-DRIVE_SPEED);
            
            telemetry.addData("Action", "Rotating Left");
            telemetry.addData("Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
        stopAllMotors();
    }
    
    /**
     * Rotate right (clockwise) using mecanum wheels
     */
    public void rotateRight(double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rightFront.setPower(-DRIVE_SPEED);
            rightBack.setPower(-DRIVE_SPEED);
            leftFront.setPower(DRIVE_SPEED);
            leftBack.setPower(DRIVE_SPEED);
            
            telemetry.addData("Action", "Rotating Right");
            telemetry.addData("Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
        stopAllMotors();
    }
    
    /**
     * Stop all motors
     */
    public void stopAllMotors() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
        
        // Brief pause to ensure motors stop
        sleep(100);
    }
    
    /**
     * Move robot to specified coordinate position using Pinpoint odometry feedback
     * Uses closed-loop control with real-time position feedback
     */
    public void moveToPosition(double targetX, double targetY) {
        // Update current position from Pinpoint
        pinpoint.update();
        currentPose = pinpoint.getPosition();
        
        double startX = currentPose.getX(DistanceUnit.INCH);
        double startY = currentPose.getY(DistanceUnit.INCH);
        
        telemetry.addData("Movement", "Moving to (%.1f, %.1f)", targetX, targetY);
        telemetry.addData("From", "(%.1f, %.1f)", startX, startY);
        telemetry.update();
        
        // Movement loop with Pinpoint feedback
        runtime.reset();
        double maxMovementTime = 10.0; // Maximum time allowed for movement
        
        while (opModeIsActive() && runtime.seconds() < maxMovementTime) {
            // Update current position from Pinpoint
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            
            double currentX = currentPose.getX(DistanceUnit.INCH);
            double currentY = currentPose.getY(DistanceUnit.INCH);
            
            // Calculate error (distance to target)
            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // Check if we've reached the target
            if (distance < POSITION_TOLERANCE) {
                telemetry.addData("Status", "Target reached!");
                telemetry.addData("Final Error", "%.2f inches", distance);
                telemetry.update();
                break;
            }
            
            // Calculate movement vector (normalized)
            double moveX = deltaX / distance;
            double moveY = deltaY / distance;
            
            // Apply proportional control - slow down as we get closer
            double speedMultiplier = Math.min(1.0, distance / 5.0); // Slow down within 5 inches
            speedMultiplier = Math.max(0.2, speedMultiplier); // Minimum 20% speed
            
            // Mecanum drive kinematics
            double frontLeftPower = (moveY + moveX) * speedMultiplier;
            double frontRightPower = (moveY - moveX) * speedMultiplier;
            double backLeftPower = (moveY - moveX) * speedMultiplier;
            double backRightPower = (moveY + moveX) * speedMultiplier;
            
            // Normalize powers to stay within [-1, 1] range
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                     Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }
            
            // Apply drive speed scaling
            leftFront.setPower(frontLeftPower * DRIVE_SPEED);
            rightFront.setPower(frontRightPower * DRIVE_SPEED);
            leftBack.setPower(backLeftPower * DRIVE_SPEED);
            rightBack.setPower(backRightPower * DRIVE_SPEED);
            
            // Real-time telemetry with Pinpoint data
            telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);
            telemetry.addData("Current", "(%.1f, %.1f)", currentX, currentY);
            telemetry.addData("Error", "%.2f inches", distance);
            telemetry.addData("Speed Multiplier", "%.1f%%", speedMultiplier * 100);
            telemetry.addData("Heading", "%.1f°", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.update();
            
            sleep(20); // Small delay for stable control loop
        }
        
        // Stop motors
        stopAllMotors();
        
        // Final position update
        pinpoint.update();
        currentPose = pinpoint.getPosition();
        
        telemetry.addData("Movement", "Completed to (%.1f, %.1f)", 
                         currentPose.getX(DistanceUnit.INCH), 
                         currentPose.getY(DistanceUnit.INCH));
        telemetry.update();
        sleep(500); // Brief pause at waypoint
    }
    
    /**
     * Move robot by specified distance using encoders
     * More precise than time-based movement
     */
    public void moveByDistance(double distanceInches, double direction) {
        // Calculate target encoder counts
        int targetCounts = (int)(distanceInches * COUNTS_PER_INCH);
        
        // Get current encoder positions
        int startLeftFront = leftFront.getCurrentPosition();
        int startRightFront = rightFront.getCurrentPosition();
        int startLeftBack = leftBack.getCurrentPosition();
        int startRightBack = rightBack.getCurrentPosition();
        
        // Calculate target positions
        int targetLeftFront = startLeftFront + (int)(targetCounts * Math.sin(Math.toRadians(direction + 45)));
        int targetRightFront = startRightFront + (int)(targetCounts * Math.sin(Math.toRadians(direction - 45)));
        int targetLeftBack = startLeftBack + (int)(targetCounts * Math.sin(Math.toRadians(direction - 45)));
        int targetRightBack = startRightBack + (int)(targetCounts * Math.sin(Math.toRadians(direction + 45)));
        
        // Set target positions
        leftFront.setTargetPosition(targetLeftFront);
        rightFront.setTargetPosition(targetRightFront);
        leftBack.setTargetPosition(targetLeftBack);
        rightBack.setTargetPosition(targetRightBack);
        
        // Switch to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Start movement
        leftFront.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        rightBack.setPower(DRIVE_SPEED);
        
        // Wait for movement to complete
        while (opModeIsActive() && 
               (leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy())) {
            
            telemetry.addData("Moving", "Distance: %.1f inches, Direction: %.1f°", 
                            distanceInches, direction);
            telemetry.addData("Encoders", "LF:%d RF:%d LB:%d RB:%d",
                            leftFront.getCurrentPosition(),
                            rightFront.getCurrentPosition(),
                            leftBack.getCurrentPosition(),
                            rightBack.getCurrentPosition());
            telemetry.update();
        }
        
        // Stop motors and return to RUN_USING_ENCODER mode
        stopAllMotors();
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Configure the goBilda Pinpoint odometry sensor
     */
    public void configurePinpoint() {
        /*
         * Set the odometry pod positions relative to the point that you want the position to be measured from.
         * The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         * Left of the center is a positive number, right of center is a negative number.
         * The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         * Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET, DistanceUnit.INCH);
        
        /*
         * Set the kind of pods used by your robot. Using goBILDA 4-bar odometry pods.
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        
        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        
        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary.
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
        
        telemetry.addData("Pinpoint", "Configured and calibrated");
        telemetry.update();
    }
    
    /**
     * Update robot position from Pinpoint odometry
     * Call this method regularly to get the latest position data
     */
    public void updatePosition() {
        pinpoint.update();
        currentPose = pinpoint.getPosition();
    }
    
    /**
     * Get current X position in inches
     */
    public double getCurrentX() {
        return currentPose.getX(DistanceUnit.INCH);
    }
    
    /**
     * Get current Y position in inches
     */
    public double getCurrentY() {
        return currentPose.getY(DistanceUnit.INCH);
    }
    
    /**
     * Get current heading in degrees
     */
    public double getCurrentHeading() {
        return currentPose.getHeading(AngleUnit.DEGREES);
    }
}