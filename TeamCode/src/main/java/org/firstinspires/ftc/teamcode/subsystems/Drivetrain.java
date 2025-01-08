package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;
//import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Robot Drivetrain
 */
public class Drivetrain implements DrivetrainConstants {
    private DcMotorEx leftDrive, rightDrive, backDrive;
    public IndicatorLight leftLight, rightLight;
    private VectorMotionProfile driveProfile;
    private MotionProfile turnProfile;
    private SimpleFeedbackController turnController;
    private double headingOffset = Math.PI / 2;
    public PoseEstimator poseEstimator;
    private double targetHeading;
    private OpticalSensor od;

    /**
     * Initializes the Drivetrain subsystem
     *
     * @param hwMap the hardwareMap
     * @param initialPose the initial Pose2D
     */
    public Drivetrain(HardwareMap hwMap, Pose2D initialPose) {
        this.targetHeading = initialPose.getHeading(AngleUnit.RADIANS);
        od = new OpticalSensor("OTOS", hwMap, DistanceUnit.INCH, AngleUnit.RADIANS);
        poseEstimator = new PoseEstimator(hwMap, initialPose, true);


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

//
//        leftEncoder = new OctoEncoder(hwMap, LEFT_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);
//        rightEncoder = new OctoEncoder(hwMap, RIGHT_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);
//        backEncoder = new OctoEncoder(hwMap, BACK_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);

        leftLight = new IndicatorLight(hwMap, "LeftLight", IndicatorLight.Colour.GREEN);
        rightLight = new IndicatorLight(hwMap, "RightLight", IndicatorLight.Colour.GREEN);

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
        boolean isManualTurning = Math.abs(turn) > TURN_DEADBAND;
//        double turnCalculated = !isManualTurning ? turnToAngle() : turnProfile.calculate(turn);
        double turnCalculated = turnProfile.calculate(turn);
        driveInput = driveProfile.calculate(driveInput.clipMagnitude(
                (lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turnCalculated)));
        double power = driveInput.magnitude();
        double angle = driveInput.angle();
        //TODO add offset and intial heading
        leftDrive.setPower(turnCalculated + power * Math.cos(angle + LEFT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS)));
        rightDrive.setPower(turnCalculated + power * Math.cos(angle + RIGHT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS)));
        backDrive.setPower(turnCalculated + power * Math.cos(angle + BACK_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS)));
    }

 
    public double turnToAngle() {
        double error = Angles.clipRadians(poseEstimator.getPose().getHeading(AngleUnit.RADIANS) - targetHeading);
        return Math.abs(error) < AUTO_ALIGN_ERROR ? 0.0 : error / 3;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

//    public int[] getMotorVelocities() {
//        return new int[]{leftEncoder.getVelocity(), rightEncoder.getVelocity(), backEncoder.getVelocity()};
//    }

    public void runDriveMotors(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backDrive.setPower(power );
    }

    public Pose2D getPose() { return poseEstimator.getPose(); }

    public void resetIMU() { od.resetHeading(); }

    public void resetElevatorEncoder() {
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getElevatorPosition() {
        return rightDrive.getCurrentPosition();
    }
}