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
        
        // Send telemetry message to indicate successful initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Speed", "%.1f", DRIVE_SPEED);
        telemetry.update();
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        
        // Run autonomous sequence
        if (opModeIsActive()) {
            
            // Example autonomous sequence demonstrating mecanum capabilities
            telemetry.addData("Path", "Starting autonomous sequence");
            telemetry.update();
            
            // Move forward for 2 seconds
            driveForward(2.0);
            
            // Strafe right for 1.5 seconds
            strafeRight(1.5);
            
            // Rotate left for 1 second
            rotateLeft(1.0);
            
            // Move backward for 1 second
            driveBackward(1.0);
            
            // Strafe left for 1.5 seconds
            strafeLeft(1.5);
            
            // Stop all motors
            stopAllMotors();
            
            telemetry.addData("Path", "Complete");
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
}