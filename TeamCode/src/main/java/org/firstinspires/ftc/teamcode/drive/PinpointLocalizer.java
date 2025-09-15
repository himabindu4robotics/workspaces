package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Road Runner localizer implementation using goBilda Pinpoint odometry sensor.
 * This class bridges the Pinpoint sensor data with Road Runner's localization system.
 */
public class PinpointLocalizer implements Localizer {
    private GoBildaPinpointDriver pinpoint;
    private Pose2d poseEstimate;
    private Pose2d poseVelocity;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
        this.poseEstimate = new Pose2d();
        this.poseVelocity = new Pose2d();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        // Set the Pinpoint position to match Road Runner's pose
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D pinpointPose = 
            new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH, pose2d.getX(), pose2d.getY(),
                AngleUnit.RADIANS, pose2d.getHeading()
            );
        pinpoint.setPosition(pinpointPose);
        this.poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
        // Update Pinpoint sensor
        pinpoint.update();
        
        // Get current pose from Pinpoint
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D pinpointPose = pinpoint.getPosition();
        
        // Convert to Road Runner pose
        Pose2d newPose = new Pose2d(
            pinpointPose.getX(DistanceUnit.INCH),
            pinpointPose.getY(DistanceUnit.INCH),
            pinpointPose.getHeading(AngleUnit.RADIANS)
        );
        
        // Calculate velocity (simple finite difference)
        // In a real implementation, you might want to use a more sophisticated approach
        if (poseEstimate != null) {
            double dt = 0.02; // Assume 20ms update rate
            Vector2d positionDelta = newPose.vec().minus(poseEstimate.vec());
            double headingDelta = newPose.getHeading() - poseEstimate.getHeading();
            
            // Normalize heading delta to [-π, π]
            while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
            while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
            
            poseVelocity = new Pose2d(
                positionDelta.div(dt),
                headingDelta / dt
            );
        }
        
        poseEstimate = newPose;
    }

    /**
     * Get the raw Pinpoint pose data
     * @return Pinpoint Pose2D object
     */
    public org.firstinspires.ftc.robotcore.external.navigation.Pose2D getRawPinpointPose() {
        return pinpoint.getPosition();
    }

    /**
     * Reset the Pinpoint sensor position and IMU
     */
    public void resetPosAndIMU() {
        pinpoint.resetPosAndIMU();
        poseEstimate = new Pose2d();
        poseVelocity = new Pose2d();
    }

    /**
     * Get Pinpoint sensor status information
     * @return Status string from Pinpoint sensor
     */
    public String getPinpointStatus() {
        // This would return sensor status if available in the Pinpoint driver
        return "Pinpoint Active";
    }
}