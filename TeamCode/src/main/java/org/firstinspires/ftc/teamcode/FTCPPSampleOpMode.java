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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode demonstrates mecanum wheel drive control with custom motor configuration.
 * 
 * Motor Configuration:
 * - RightFront: Forward direction
 * - RightBack: Forward direction  
 * - LeftFront: Reverse direction
 * - LeftBack: Reverse direction
 * 
 * Speed is set to 0.1 (one tenth of maximum speed) for precise control.
 * 
 * The code provides basic mecanum drive movements:
 * - Forward/Backward
 * - Strafe Left/Right
 * - Rotate Left/Right
 * - Diagonal movements
 */

@Autonomous(name="FTCPP Sample OpMode", group="Custom")
public class FTCPPSampleOpMode extends LinearOpMode {

    /* Declare OpMode members */
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    // Motor speed constant - one tenth of maximum speed
    static final double DRIVE_SPEED = 0.1;
    
    // Position tracking variables
    private double currentX = 0.0;  // Current X position in inches
    private double currentY = 0.0;  // Current Y position in inches
    private double currentHeading = 0.0;  // Current heading in degrees
    
    // Movement constants for coordinate calculations
    static final double COUNTS_PER_MOTOR_REV = 1440.0;  // Motor encoder counts per revolution
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // External gear reduction
    static final double WHEEL_DIAMETER_INCHES = 4.0;    // Wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 
                                         (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double ROBOT_WIDTH_INCHES = 18.0;      // Distance between left and right wheels
    
    @Override
    public void runOpMode() {
        
        // Initialize the drive system variables
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        
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
        
        // Reset encoders for position tracking
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize position tracking
        currentX = 0.0;
        currentY = 0.0;
        currentHeading = 0.0;
        
        // Send telemetry message to indicate successful initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Speed", "%.1f", DRIVE_SPEED);
        telemetry.addData("Starting Position", "(%.1f, %.1f)", currentX, currentY);
        telemetry.update();
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        
        // Run coordinate-based autonomous sequence
        if (opModeIsActive()) {
            
            // Coordinate-based autonomous sequence
            telemetry.addData("Path", "Starting coordinate-based autonomous sequence");
            telemetry.addData("Target Path", "(0,0) -> (20,20) -> (40,20)");
            telemetry.update();
            
            // Robot starts at position (0,0)
            telemetry.addData("Current Position", "(%.1f, %.1f)", currentX, currentY);
            telemetry.update();
            sleep(1000);
            
            // Move to position (20,20)
            moveToPosition(20.0, 20.0);
            
            // Pause at waypoint
            sleep(1000);
            
            // Move to position (40,20)
            moveToPosition(40.0, 20.0);
            
            // Stop all motors
            stopAllMotors();
            
            telemetry.addData("Path", "Complete");
            telemetry.addData("Final Position", "(%.1f, %.1f)", currentX, currentY);
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
     * Move robot to specified coordinate position
     * Uses simple point-to-point movement with mecanum drive
     */
    public void moveToPosition(double targetX, double targetY) {
        telemetry.addData("Movement", "Moving to (%.1f, %.1f)", targetX, targetY);
        telemetry.addData("From", "(%.1f, %.1f)", currentX, currentY);
        telemetry.update();
        
        // Calculate movement deltas
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        if (distance < 0.5) {
            // Already at target position
            telemetry.addData("Status", "Already at target position");
            telemetry.update();
            return;
        }
        
        // Calculate movement time based on distance and speed
        // Assuming 12 inches per second at DRIVE_SPEED = 0.1
        double movementSpeed = 12.0 * DRIVE_SPEED; // inches per second
        double movementTime = distance / movementSpeed;
        
        // Normalize movement vector
        double moveX = deltaX / distance;
        double moveY = deltaY / distance;
        
        // Execute movement using mecanum drive kinematics
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < movementTime)) {
            // Mecanum drive: combine X and Y movement
            double frontLeftPower = moveY + moveX;
            double frontRightPower = moveY - moveX;
            double backLeftPower = moveY - moveX;
            double backRightPower = moveY + moveX;
            
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
            
            // Update telemetry
            telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);
            telemetry.addData("Progress", "%.1f%% (%.1fs/%.1fs)", 
                            (runtime.seconds() / movementTime) * 100, 
                            runtime.seconds(), movementTime);
            telemetry.addData("Distance Remaining", "%.1f inches", 
                            distance * (1.0 - runtime.seconds() / movementTime));
            telemetry.update();
        }
        
        // Stop motors
        stopAllMotors();
        
        // Update current position
        currentX = targetX;
        currentY = targetY;
        
        telemetry.addData("Movement", "Reached position (%.1f, %.1f)", currentX, currentY);
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
     * Update robot position based on encoder readings
     * Simple odometry for position tracking
     */
    public void updatePosition() {
        // This is a simplified position update
        // In a real implementation, you would use more sophisticated odometry
        // incorporating all four wheel encoders and IMU data
        
        // For now, we'll update position based on the movement commands
        // This is handled in the moveToPosition method
    }
}