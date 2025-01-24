package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final IntSupplier elevatorPosition;
    public final PIDController pidController;
    private double setpoint;
    private ELEVATOR_STATE state;

    public enum ELEVATOR_STATE
    {
        MANUAL,
        GO_TO_POS,
        STOPPED
    }

    /**
     * Initializes the Arm
     *
     * @param hardwareMap the hardware map
     */
    public Elevator(HardwareMap hardwareMap, IntSupplier elevatorPosition) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "LeftClimberMotor");
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "RightClimberMotor");
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);


        setpoint = 0.0;
        pidController = new PIDController(SLIDE_P, 0.0, 0.0);
        this.elevatorPosition = elevatorPosition;
        this.state = ELEVATOR_STATE.GO_TO_POS;
    }

    public void periodic() {
        switch( state )
        {
            case MANUAL:
                this.setpoint = this.getEncoderPosition();
                break;
            case GO_TO_POS:
                slideRawPower(-pidController.calculate(this.getEncoderPosition(), this.setpoint));
                break;
            case STOPPED:
            default:
                slideRawPower(0);
        }
    }

    public int getEncoderPosition() { return elevatorPosition.getAsInt(); }


    public void setSetpoint(double setpoint) {
        this.state = ELEVATOR_STATE.GO_TO_POS;
        this.setpoint = setpoint;
    }

    public void manualControl(double power)
    {
        this.state = ELEVATOR_STATE.MANUAL;
        slideRawPower(power);
    }

    public void setPIDCoefficients(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }

    private void slideRawPower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    public void stopElevator()
    {
        state = ELEVATOR_STATE.STOPPED;
    }

    public ELEVATOR_STATE getState()
    {
        return state;
    }

    public void setState(ELEVATOR_STATE state)
    {
        this.state = state;
    }
}