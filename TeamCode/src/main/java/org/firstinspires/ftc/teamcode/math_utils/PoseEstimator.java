package org.firstinspires.ftc.teamcode.math_utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.subsystems.components.LimelightComponent;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;

import java.util.Optional;

/**
 * The Robot Pose (x, y, theta)
 */
public class PoseEstimator implements DrivetrainConstants {
    public Pose2D initialPose;

    public Pose2D robotPose;

    public LimelightComponent limelight;

    public OpticalSensor otos;

    public DcMotor odomX, odomY;

    public double currentXTicks = 0.0;
    public double currentYTicks = 0.0;
    public double previousHeading = 0.0;
    public boolean useLL;

    public boolean isUsingLL;

    /**
     * The Field-Relative Robot Pose.
     * @param hwMap OpMode Hardware Map passthrough for LL, OTOS, and Gyro initialization.
     * @param initialPose Pose2D for robot's initial field-relative position.
     */
    public PoseEstimator(HardwareMap hwMap, Pose2D initialPose, boolean useLL, boolean suppressOTOSReset) {
        odomX = hwMap.get(DcMotor.class, "RightDriveMotor");
        odomY = hwMap.get(DcMotor.class, "BackDriveMotor");
        odomX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odomX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odomY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odomY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.useLL = useLL;
        Optional<Point> LLPose = Optional.empty();
        otos = new OpticalSensor("OTOS", hwMap, DistanceUnit.METER, AngleUnit.DEGREES, suppressOTOSReset);
        if (useLL) {
            limelight = new LimelightComponent(hwMap, otos::getYawRate);
            limelight.updateWithPose(initialPose.getHeading(AngleUnit.DEGREES));
            LLPose = limelight.getRobotPose();
        }

        //If the limelight can localize at startup, use that for the initial pose.
        if (useLL && LLPose.isPresent() ) {
            this.initialPose = new Pose2D(
                    DistanceUnit.METER,
                    LLPose.get().x,
                    LLPose.get().y,
                    AngleUnit.DEGREES,
                    initialPose.getHeading(AngleUnit.DEGREES)
            );
        } else {
            this.initialPose = new Pose2D(
                    DistanceUnit.METER,
                    initialPose.getX(DistanceUnit.METER),
                    initialPose.getY(DistanceUnit.METER),
                    AngleUnit.DEGREES,
                    initialPose.getHeading(AngleUnit.DEGREES)
            );

            this.robotPose = this.initialPose;

        }
    }

    public Pose2D getPose() { return robotPose; }
    // run through transform FO for the actual one
    public Vector getOdomDeltas(){
        Vector deltas = new Vector(( (currentXTicks - odomX.getCurrentPosition()) - ((previousHeading - getFieldHeading(AngleUnit.DEGREES)) * TICKS_PER_DEGREE_X)) * X_TICK_MULTIPLIER,
                ( (currentYTicks - odomY.getCurrentPosition()) - ((previousHeading - getFieldHeading(AngleUnit.DEGREES)) * TICKS_PER_DEGREE_Y)) * Y_TICK_MULTIPLIER);
        currentXTicks = odomX.getCurrentPosition();
        currentYTicks = odomY.getCurrentPosition();
        previousHeading = getFieldHeading(AngleUnit.DEGREES);

        return deltas;

    }
    public boolean isUsingLL() { return isUsingLL; }

    public void update() {
        otos.update();

        if (useLL && robotPose != null) limelight.updateWithPose(robotPose.getHeading(AngleUnit.DEGREES));

        Optional<Point> LLEstimate = Optional.empty();
        if (useLL) LLEstimate = limelight.getRobotPose();

        isUsingLL = LLEstimate.isPresent();

        LLEstimate.ifPresent(point -> robotPose = new Pose2D(
                DistanceUnit.METER,
                point.x,
                point.y,
                AngleUnit.DEGREES,
                getFieldHeading(AngleUnit.DEGREES)));

        if( !LLEstimate.isPresent() ) {
            Vector transformedDeltas = transformFieldOriented(otos.getPosDeltas());
            robotPose = new Pose2D(
                    DistanceUnit.METER,
                    robotPose.getX(DistanceUnit.METER) + transformedDeltas.x,
                    robotPose.getY(DistanceUnit.METER) + transformedDeltas.y,
                    AngleUnit.DEGREES,
                    getFieldHeading(AngleUnit.DEGREES)
            );
        }
    }

    public double getInitialHeading(AngleUnit unit) {
        return initialPose.getHeading(unit);
    }

    private double getFieldHeading(AngleUnit unit) {
        if (unit == AngleUnit.DEGREES) {
            return Angles.clipDegrees(initialPose.getHeading(AngleUnit.DEGREES) + otos.getHeading());
        } else {
            return Angles.clipRadians(initialPose.getHeading(AngleUnit.RADIANS) + Math.toRadians(otos.getHeading()));
        }
    }

    /**
     * Takes in a vector that is robot-oriented (such as OTOS position/deltas) and converts it to
     * field-oriented using the field heading calculated from the gyro and inputted initialHeading.
     * @param inputVector The robot-oriented vector.
     * @return The field-oriented vector.
     */
    private Vector transformFieldOriented(Vector inputVector) {
        return new Vector(
                (Math.cos(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.x) - (Math.sin(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.y),
                (Math.sin(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.x) + (Math.cos(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.y)
        );
    }

    /**
     * Method for testing the robot to field transformation using the OTOS data (not deltas)
     * @return The transformed Vector.
     */
    public Pose2D debugTransform() {
        Vector otosPos = otos.getPosition();
        Vector transformedPos = transformFieldOriented(otosPos);

        return new Pose2D(DistanceUnit.METER, initialPose.getX(DistanceUnit.METER) + transformedPos.x, initialPose.getY(DistanceUnit.METER) + transformedPos.y, AngleUnit.DEGREES, getFieldHeading(AngleUnit.DEGREES));
    }

    public void resetOTOSIMU(){
        otos.resetHeading();
    }

}