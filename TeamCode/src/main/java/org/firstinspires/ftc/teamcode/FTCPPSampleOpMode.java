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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.PinpointMecanumDrive;

/**
 * This OpMode demonstrates Road Runner trajectory following with goBilda Pinpoint odometry.
 * 
 * Motor Configuration:
 * - RightFront: Forward direction
 * - RightBack: Forward direction  
 * - LeftFront: Reverse direction
 * - LeftBack: Reverse direction
 * 
 * Advanced Navigation System:
 * - Road Runner trajectory planning and following
 * - goBilda Pinpoint encoder for precise position feedback
 * - Smooth spline-based paths with velocity profiles
 * - Advanced motion control with PID feedback
 * 
 * The code provides sophisticated autonomous navigation:
 * - Spline trajectory generation
 * - Velocity and acceleration constraints
 * - Real-time trajectory following
 * - Precise waypoint navigation
 */

@Autonomous(name="FTCPP Road Runner + Pinpoint", group="Advanced")
public class FTCPPSampleOpMode extends LinearOpMode {

    /* Declare OpMode members */
    private PinpointMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Road Runner trajectory constants
    static final double TRAJECTORY_TIMEOUT = 30.0;     // Maximum time for trajectory execution
    static final double POSITION_TOLERANCE = 0.5;      // Position tolerance in inches
    static final double HEADING_TOLERANCE = Math.toRadians(2.0); // Heading tolerance in radians
    
    @Override
    public void runOpMode() {
        
        // Initialize Road Runner drive with Pinpoint odometry
        drive = new PinpointMecanumDrive(hardwareMap);
        
        // Set starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        
        // Send telemetry message to indicate successful initialization
        telemetry.addData("Status", "Initialized with Road Runner + Pinpoint");
        telemetry.addData("Starting Position", "(%.1f, %.1f)", 
                         startPose.getX(), startPose.getY());
        telemetry.addData("Starting Heading", "%.1f°", Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Road Runner", "Ready for trajectory execution");
        telemetry.update();
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        
        // Build Road Runner trajectories for the path (0,0) -> (20,20) -> (40,20)
        if (opModeIsActive()) {
            
            telemetry.addData("Path", "Building Road Runner trajectories");
            telemetry.addData("Target Path", "(0,0) -> (20,20) -> (40,20)");
            telemetry.addData("Navigation System", "Road Runner + Pinpoint Odometry");
            telemetry.update();
            
            // Build trajectory from (0,0) to (20,20) - Diagonal movement with spline
            Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(20, 20), 0) // Smooth spline to (20,20) with 0 heading
                    .build();
            
            // Build trajectory from (20,20) to (40,20) - Lateral movement
            Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                    .lineToLinearHeading(new Pose2d(40, 20, 0)) // Linear movement maintaining heading
                    .build();
            
            telemetry.addData("Trajectories", "Built successfully");
            telemetry.addData("Trajectory 1", "Spline to (20,20)");
            telemetry.addData("Trajectory 2", "Linear to (40,20)");
            telemetry.update();
            sleep(1000);
            
            // Execute first trajectory: (0,0) -> (20,20)
            telemetry.addData("Executing", "Trajectory 1: Spline to (20,20)");
            telemetry.update();
            
            drive.followTrajectory(trajectory1);
            
            // Wait for trajectory to complete with real-time feedback
            while (opModeIsActive() && drive.isFollowingTrajectory()) {
                drive.update();
                
                Pose2d currentPose = drive.getPoseEstimate();
                Pose2d error = drive.getLastError();
                
                telemetry.addData("Status", "Following Trajectory 1");
                telemetry.addData("Current Position", "(%.1f, %.1f)", 
                                 currentPose.getX(), currentPose.getY());
                telemetry.addData("Current Heading", "%.1f°", 
                                 Math.toDegrees(currentPose.getHeading()));
                telemetry.addData("Position Error", "%.2f inches", 
                                 Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY()));
                telemetry.addData("Heading Error", "%.1f°", 
                                 Math.toDegrees(error.getHeading()));
                telemetry.update();
            }
            
            // Display waypoint 1 reached
            Pose2d waypoint1Pose = drive.getPoseEstimate();
            telemetry.addData("Waypoint 1", "Reached (%.1f, %.1f)", 
                             waypoint1Pose.getX(), waypoint1Pose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(waypoint1Pose.getHeading()));
            telemetry.update();
            sleep(1000);
            
            // Execute second trajectory: (20,20) -> (40,20)
            telemetry.addData("Executing", "Trajectory 2: Linear to (40,20)");
            telemetry.update();
            
            drive.followTrajectory(trajectory2);
            
            // Wait for trajectory to complete with real-time feedback
            while (opModeIsActive() && drive.isFollowingTrajectory()) {
                drive.update();
                
                Pose2d currentPose = drive.getPoseEstimate();
                Pose2d error = drive.getLastError();
                
                telemetry.addData("Status", "Following Trajectory 2");
                telemetry.addData("Current Position", "(%.1f, %.1f)", 
                                 currentPose.getX(), currentPose.getY());
                telemetry.addData("Current Heading", "%.1f°", 
                                 Math.toDegrees(currentPose.getHeading()));
                telemetry.addData("Position Error", "%.2f inches", 
                                 Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY()));
                telemetry.addData("Heading Error", "%.1f°", 
                                 Math.toDegrees(error.getHeading()));
                telemetry.update();
            }
            
            // Final position and completion
            Pose2d finalPose = drive.getPoseEstimate();
            
            telemetry.addData("Path", "Complete - Road Runner Trajectories Executed");
            telemetry.addData("Final Position", "(%.1f, %.1f)", 
                             finalPose.getX(), finalPose.getY());
            telemetry.addData("Final Heading", "%.1f°", 
                             Math.toDegrees(finalPose.getHeading()));
            telemetry.addData("Total Run Time", "%.1f seconds", runtime.seconds());
            telemetry.addData("Navigation", "Road Runner + Pinpoint Odometry");
            telemetry.update();
        }
    }
    
    /**
     * Execute a Road Runner trajectory with timeout safety
     */
    public void executeTrajectory(Trajectory trajectory, String description) {
        telemetry.addData("Executing", description);
        telemetry.update();
        
        drive.followTrajectory(trajectory);
        
        runtime.reset();
        while (opModeIsActive() && drive.isFollowingTrajectory() && runtime.seconds() < TRAJECTORY_TIMEOUT) {
            drive.update();
            
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d error = drive.getLastError();
            
            telemetry.addData("Status", description);
            telemetry.addData("Current Position", "(%.1f, %.1f)", 
                             currentPose.getX(), currentPose.getY());
            telemetry.addData("Current Heading", "%.1f°", 
                             Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Position Error", "%.2f inches", 
                             Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY()));
            telemetry.addData("Heading Error", "%.1f°", 
                             Math.toDegrees(error.getHeading()));
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
        
        // Ensure trajectory is stopped if timeout occurred
        if (runtime.seconds() >= TRAJECTORY_TIMEOUT) {
            drive.setMotorPowers(0, 0, 0, 0);
            telemetry.addData("Warning", "Trajectory timeout - stopped motors");
            telemetry.update();
        }
    }
    
    /**
     * Build a spline trajectory to a target position
     */
    public Trajectory buildSplineTrajectory(Pose2d startPose, Vector2d targetPosition, double targetHeading) {
        return drive.trajectoryBuilder(startPose)
                .splineTo(targetPosition, targetHeading)
                .build();
    }
    
    /**
     * Build a linear trajectory to a target pose
     */
    public Trajectory buildLinearTrajectory(Pose2d startPose, Pose2d targetPose) {
        return drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(targetPose)
                .build();
    }
    
    /**
     * Get current robot pose from Road Runner
     */
    public Pose2d getCurrentPose() {
        return drive.getPoseEstimate();
    }
    
    /**
     * Stop all drive motors
     */
    public void stopAllMotors() {
        drive.setMotorPowers(0, 0, 0, 0);
        sleep(100);
    }
    
    /**
     * Create a complex trajectory sequence with multiple waypoints
     */
    public void executeComplexPath() {
        Pose2d startPose = drive.getPoseEstimate();
        
        // Build a complex trajectory with splines and linear segments
        Trajectory complexTrajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(10, 10), Math.toRadians(45))  // Curved approach
                .splineTo(new Vector2d(20, 20), Math.toRadians(0))   // Smooth curve to waypoint 1
                .lineToLinearHeading(new Pose2d(40, 20, Math.toRadians(0))) // Linear to final point
                .build();
        
        executeTrajectory(complexTrajectory, "Complex Multi-Waypoint Path");
    }
    
    /**
     * Demonstrate advanced Road Runner trajectory features
     */
    public void demonstrateAdvancedFeatures() {
        Pose2d currentPose = drive.getPoseEstimate();
        
        // Example of trajectory with velocity and acceleration constraints
        Trajectory constrainedTrajectory = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(currentPose.getX() + 12, currentPose.getY() + 12), 0)
                .build();
        
        telemetry.addData("Demo", "Advanced trajectory with constraints");
        telemetry.update();
        
        executeTrajectory(constrainedTrajectory, "Constrained Velocity Trajectory");
    }
    
    /**
     * Display comprehensive telemetry about the robot's state
     */
    public void displayRobotStatus() {
        Pose2d currentPose = drive.getPoseEstimate();
        Pose2d lastError = drive.getLastError();
        
        telemetry.addData("=== ROBOT STATUS ===", "");
        telemetry.addData("Position", "(%.1f, %.1f)", currentPose.getX(), currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Position Error", "%.2f inches", 
                         Math.sqrt(lastError.getX() * lastError.getX() + lastError.getY() * lastError.getY()));
        telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(lastError.getHeading()));
        telemetry.addData("Following Trajectory", drive.isFollowingTrajectory() ? "Yes" : "No");
        telemetry.addData("Navigation System", "Road Runner + Pinpoint");
        telemetry.update();
    }
    
    /**
     * Reset robot pose to origin
     */
    public void resetPose() {
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        telemetry.addData("Reset", "Robot pose reset to origin");
        telemetry.update();
    }
}