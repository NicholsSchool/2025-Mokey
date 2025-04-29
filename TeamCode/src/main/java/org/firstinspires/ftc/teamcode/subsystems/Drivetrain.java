package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
//import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain implements DrivetrainConstants {
    private final DcMotorEx leftDrive, rightDrive, backDrive;
    public IndicatorLight leftLight, rightLight;
    private final VectorMotionProfile driveProfile;
    private final MotionProfile turnProfile;
    private final PIDController turnController;
    public PoseEstimator poseEstimator;
    private double targetHeading;
    private final double fieldOrientedForward;
    private final PIDController drivePID;
    public double goToPosDistance;
    private Pose2D setpoint = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0);
    private Vector PIDDriveVector = new Vector(0 ,0);

    /**
     * Initializes the Drivetrain subsystem
     *
     * @param hwMap the hardwareMap
     * @param initialPose the initial Pose2D
     * @param fieldOrientedForward the field direction that the robot drives when turn angle is 0 (in degrees)
     *                             !IMPORTANT! For autos, this should be 270.
     */
    public Drivetrain(HardwareMap hwMap, Pose2D initialPose, double fieldOrientedForward, boolean useLL, boolean suppressHeadingReset) {
        this.targetHeading = initialPose.getHeading(AngleUnit.RADIANS);
        this.fieldOrientedForward = Math.toRadians(fieldOrientedForward) - Math.PI / 2;
        poseEstimator = new PoseEstimator(hwMap, initialPose, useLL, suppressHeadingReset);

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

        drivePID = new PIDController( GO_TO_POS_P, GO_TO_POS_I, GO_TO_POS_D);

        leftLight = new IndicatorLight(hwMap, "LeftLight", IndicatorLight.Colour.RED);
        rightLight = new IndicatorLight(hwMap, "RightLight", IndicatorLight.Colour.RED);

        driveProfile = new VectorMotionProfile(DRIVE_PROFILE_SPEED);
        turnProfile = new MotionProfile(TURN_PROFILE_SPEED, TURN_PROFILE_MAX);
        turnController = new PIDController(AUTO_ALIGN_P, 0.0, 0.0);
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

    public double newTurnToAngle() {
        double error = Angles.clipRadians(poseEstimator.getPose().getHeading(AngleUnit.RADIANS) - targetHeading);
        return turnController.calculate(error, 0);
    }

    public AutoUtil.AutoActionState driveToPose(Pose2D targetPose, boolean lowGear){
        this.setpoint = targetPose;

        if( poseEstimator.getPose() == null )
            return AutoUtil.AutoActionState.RUNNING;

        Vector diffVector = new Vector(targetPose.getX(DistanceUnit.INCH) - poseEstimator.getPose().getX(DistanceUnit.INCH),
                targetPose.getY(DistanceUnit.INCH) - poseEstimator.getPose().getY(DistanceUnit.INCH));

        goToPosDistance = diffVector.magnitude();

        if ( diffVector.magnitude() < DRIVE_SETPOINT_THRESHOLD &&
                Math.abs( this.getPose().getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES) ) < TURN_SETPOINT_THRESHOLD ) {
            return AutoUtil.AutoActionState.FINISHED;
        }

        double driveMagnitude = drivePID.calculate(diffVector.magnitude(), 0);
        double driveAngle = diffVector.angle();

        Vector driveInput = new Vector(

                driveMagnitude * Math.cos(driveAngle),
                driveMagnitude * Math.sin(driveAngle)
        );

        driveInput.clipMagnitude(1.0);

        PIDDriveVector = driveInput;

        setTargetHeading(targetPose.getHeading(AngleUnit.RADIANS));

        autoDrive(driveInput, newTurnToAngle(), lowGear);

        return AutoUtil.AutoActionState.RUNNING;
    }
//
//    public AutoUtil.AutoActionState driveToPose(Pose2D targetPose, boolean lowGear) {
//        Vector driveInput = new Vector(targetPose.getX(DistanceUnit.INCH) - poseEstimator.getPose().getX(DistanceUnit.INCH),
//             targetPose.getY(DistanceUnit.INCH) - poseEstimator.getPose().getY(DistanceUnit.INCH));
//
//        if ( driveInput.magnitude() < DRIVE_SETPOINT_THRESHOLD &&
//                Math.abs( this.getPose().getHeading(AngleUnit.DEGREES) - targetPose.getHeading(AngleUnit.DEGREES) ) > TURN_SETPOINT_THRESHOLD ) {
//            return AutoUtil.AutoActionState.FINISHED;
//        }
//
//        setTargetHeading(targetPose.getHeading(AngleUnit.RADIANS));
//    }

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
                .setStroke(poseEstimator.isUsingLL ? "Green" : "Red")
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
                )
                .setStroke("Purple")
                .strokeCircle(setpoint.getX(DistanceUnit.INCH), setpoint.getY(DistanceUnit.INCH), DRIVE_SETPOINT_THRESHOLD)
                .setStroke("Cyan")
                .strokeLine(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        poseEstimator.getPose().getX(DistanceUnit.INCH) + PIDDriveVector.x,
                        poseEstimator.getPose().getY(DistanceUnit.INCH) + PIDDriveVector.y
                );
        dashboard.sendTelemetryPacket(packet);
    }

    public double getYawVelo() {
        return poseEstimator.otos.getYawRate();
    }

    public Pose2D getPose() { return poseEstimator.getPose(); }

    public void resetIMU() { poseEstimator.resetOTOSIMU(); }

    public void stop() {
        runDriveMotors(0);
    }
}