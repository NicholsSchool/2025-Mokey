package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
//import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain implements DrivetrainConstants {
    private final DcMotorEx leftDrive, rightDrive, backDrive;
    //public IndicatorLight leftLight, rightLight;
    private final VectorMotionProfile driveProfile;
    private final MotionProfile turnProfile;
    private final SimpleFeedbackController turnController;
    public PoseEstimator poseEstimator;
    private double targetHeading;
    private final double fieldOrientedForward;
    private final OpticalSensor od;

    /**
     * Initializes the Drivetrain subsystem
     *
     * @param hwMap the hardwareMap
     * @param initialPose the initial Pose2D
     * @param fieldOrientedForward the field direction that the robot drives when turn angle is 0 (in degrees)
     *                             !IMPORTANT! For autos, this should be 270.
     */
    public Drivetrain(HardwareMap hwMap, Pose2D initialPose, double fieldOrientedForward, boolean useLL) {
        this.targetHeading = initialPose.getHeading(AngleUnit.RADIANS);
        this.fieldOrientedForward = Math.toRadians(fieldOrientedForward) - Math.PI / 2;
        od = new OpticalSensor("OTOS", hwMap, DistanceUnit.INCH, AngleUnit.RADIANS);
        poseEstimator = new PoseEstimator(hwMap, initialPose, useLL);

        leftDrive = hwMap.get(DcMotorEx.class, "LeftDriveMotor");
        rightDrive = hwMap.get(DcMotorEx.class, "RightDriveMotor");
        backDrive = hwMap.get(DcMotorEx.class, "BackDriveMotor");

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
        rightDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
        backDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);

//        leftLight = new IndicatorLight(hwMap, "LeftLight", IndicatorLight.Colour.GREEN);
//        rightLight = new IndicatorLight(hwMap, "RightLight", IndicatorLight.Colour.GREEN);

        driveProfile = new VectorMotionProfile(DRIVE_PROFILE_SPEED);
        turnProfile = new MotionProfile(TURN_PROFILE_SPEED, TURN_PROFILE_MAX);
        turnController = new SimpleFeedbackController(AUTO_ALIGN_P);
    }

    public void update() {
        poseEstimator.update();
    }

    /**
     * Drives the robot field oriented
     *
     * @param driveInput the (x, y) input
     * @param turn the turning input
     * @param lowGear whether to put the robot to virtual low gear
     */
    public void drive(Vector driveInput, double turn, boolean lowGear) {
        double turnCalculated = turnProfile.calculate(turn);
        driveInput = driveProfile.calculate(driveInput.clipMagnitude(
                (lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turnCalculated)));
        double power = driveInput.magnitude();
        double angle = driveInput.angle();
        leftDrive.setPower(turnCalculated + power * Math.cos(angle + LEFT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS) + fieldOrientedForward));
        rightDrive.setPower(turnCalculated + power * Math.cos(angle + RIGHT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS) + fieldOrientedForward));
        backDrive.setPower(turnCalculated + power * Math.cos(angle + BACK_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS) + fieldOrientedForward));
    }

    public void autoDrive(Vector driveInput, double turn, boolean lowGear) {
        double turnCalculated = turnProfile.calculate(turn);
        driveInput.clipMagnitude((lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turnCalculated));
        double power = driveInput.magnitude();
        double angle = driveInput.angle();
        leftDrive.setPower(turnCalculated + power * Math.cos(angle + LEFT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS) + fieldOrientedForward));
        rightDrive.setPower(turnCalculated + power * Math.cos(angle + RIGHT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS) + fieldOrientedForward));
        backDrive.setPower(turnCalculated + power * Math.cos(angle + BACK_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS) + fieldOrientedForward));
    }

    public double turnToAngle() {
        double error = Angles.clipRadians(poseEstimator.getPose().getHeading(AngleUnit.RADIANS) - targetHeading);
        return Math.abs(error) < AUTO_ALIGN_ERROR ? 0.0 : error / 3;
    }

    public AutoUtil.AutoActionState driveToPose(Pose2D targetPose, boolean lowGear){
        Vector driveInput = new Vector(targetPose.getX(DistanceUnit.INCH) - poseEstimator.getPose().getX(DistanceUnit.INCH),
                targetPose.getY(DistanceUnit.INCH) - poseEstimator.getPose().getY(DistanceUnit.INCH));

        if ( driveInput.magnitude() < DRIVE_SETPOINT_THRESHOLD &&
                Math.abs( this.getPose().getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES) ) > TURN_SETPOINT_THRESHOLD ) {
            return AutoUtil.AutoActionState.FINISHED;
        }

        setTargetHeading(targetPose.getHeading(AngleUnit.RADIANS));

        if( driveInput.magnitude() < DRIVE_SETPOINT_THRESHOLD )
            return AutoUtil.AutoActionState.RUNNING;

        driveInput.scaleMagnitude(-DRIVE_PROPORTIONAL * Math.pow(driveInput.magnitude(), 2));
        autoDrive(driveInput,turnToAngle(), lowGear);

        return AutoUtil.AutoActionState.RUNNING;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public void runDriveMotors(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backDrive.setPower(power );
    }

    public void sendDashboardPacket(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setAlpha(0.7)
                .setStrokeWidth(1)
                .setStroke("Red")
                .strokeCircle(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        9
                )
                .setStroke("Green")
                .strokeLine(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        poseEstimator.getPose().getX(DistanceUnit.INCH) + (9 * Math.cos(poseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                        poseEstimator.getPose().getY(DistanceUnit.INCH) + (9 * Math.sin(poseEstimator.getPose().getHeading(AngleUnit.RADIANS)))
                );
        dashboard.sendTelemetryPacket(packet);
    }

    public Pose2D getPose() { return poseEstimator.getPose(); }

    public void resetIMU() { od.resetHeading(); }

    public void resetElevatorEncoder() {
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getElevatorPosition() {
        return backDrive.getCurrentPosition();
    }
}