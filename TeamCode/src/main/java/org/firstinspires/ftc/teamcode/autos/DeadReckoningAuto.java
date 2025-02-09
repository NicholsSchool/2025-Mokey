package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.concurrent.TimeUnit;

/**
 * Testing Auto for Lerp
 */
@Autonomous(name="Contest Auto", group="Testing")
public class DeadReckoningAuto extends LinearOpMode{

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 0, 0, 0, false);
        drivetrain.setFloat();
        ElapsedTime time = new ElapsedTime();
        Elevator elevator = new Elevator(hardwareMap, drivetrain::getElevatorPosition);

        waitForStart();

        drivetrain.setTargetHeading(drivetrain.getPose().angle);
        elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY);
        while(time.time(TimeUnit.SECONDS) < 5.0){
            drivetrain.drive(new Vector(0.0,0.7), 0, true);
            elevator.periodic();
            drivetrain.update();
        }
        time.reset();
        elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_PULL);
        while(time.time(TimeUnit.SECONDS) < 1.5){
            elevator.periodic();
        }
        time.reset();
        elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY);
        while(time.time(TimeUnit.SECONDS) < 1.5){
            elevator.periodic();
        }
        time.reset();
        while(time.time(TimeUnit.SECONDS) < 3.0){
            drivetrain.drive(new Vector(0.8,-0.5), 0, true);
            drivetrain.update();
        }
        time.reset();
        elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO);
        while(time.time(TimeUnit.SECONDS) < 2){
            elevator.periodic();
        }
    }
}
