package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for Road Runner with Pinpoint odometry integration.
 */
@Config
public class PinpointMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<DcMotorEx> motors;

    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor batteryVoltageSensor;

    public PinpointMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize motors with the same names as FTCPPSampleOpMode
        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "RightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // Set motor directions to match FTCPPSampleOpMode configuration
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Initialize Pinpoint odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // Set initial pose
        setPoseEstimate(new Pose2d(0, 0, 0));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, getVelocityConstraint(), getAccelerationConstraint());
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, getVelocityConstraint(), getAccelerationConstraint());
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, getVelocityConstraint(), getAccelerationConstraint());
    }

    public TrajectoryFollower getFollower() { return follower; }

    public TrajectoryVelocityConstraint getVelocityConstraint() {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
    }

    public TrajectoryAccelerationConstraint getAccelerationConstraint() {
        return new ProfileAccelerationConstraint(MAX_ACCEL);
    }

    @Override
    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    @Override
    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    @Override
    public Pose2d getLastError() {
        return follower.getLastError();
    }

    @Override
    public void update() {
        updatePoseEstimate();
        DriveSignal signal = follower.update(getPoseEstimate());
        if (signal != null) setDriveSignal(signal);
    }

    @Override
    public void updatePoseEstimate() {
        // Update Pinpoint odometry
        pinpoint.update();
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D pinpointPose = pinpoint.getPosition();
        
        // Convert Pinpoint pose to Road Runner pose
        Pose2d roadRunnerPose = new Pose2d(
            pinpointPose.getX(DistanceUnit.INCH),
            pinpointPose.getY(DistanceUnit.INCH),
            pinpointPose.getHeading(AngleUnit.RADIANS)
        );
        
        setPoseEstimate(roadRunnerPose);
    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                driveSignal.getVel(),
                TRACK_WIDTH,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER
        );
        double[] powers = new double[4];
        for (int i = 0; i < 4; i++) {
            powers[i] = velocities.get(i) / MAX_VEL;
        }
        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    @Override
    public void setDrivePower(Pose2d drivePower) {
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
                drivePower,
                1,
                1,
                LATERAL_MULTIPLIER
        );
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double leftFront, double leftBack, double rightBack, double rightFront) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
        this.rightFront.setPower(rightFront);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private void configurePinpoint() {
        // Configure Pinpoint with the same settings as FTCPPSampleOpMode
        pinpoint.setOffsets(-3.31, -6.61, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    // Drive constants - these should be tuned for your specific robot
    public static final double TICKS_PER_REV = 1440;
    public static final double MAX_RPM = 312;
    public static final double WHEEL_RADIUS = 2; // in
    public static final double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static final double TRACK_WIDTH = 18; // in

    public static final double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static final double kA = 0;
    public static final double kStatic = 0;

    public static final double MAX_VEL = 30;
    public static final double MAX_ACCEL = 30;
    public static final double MAX_ANG_VEL = Math.toRadians(180);
    public static final double MAX_ANG_ACCEL = Math.toRadians(180);

    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}