package org.firstinspires.ftc.teamcode.subsystems.components;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math_utils.Point;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class LimelightComponent {
    
    private final Limelight3A limelight3A;
    private final DoubleSupplier yawRate;

    /**
     * Constructs a limelight subsytem.
     */
    public LimelightComponent(final HardwareMap hwMap, DoubleSupplier yawRate )
    {
        this.limelight3A = hwMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(11);
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

        this.yawRate = yawRate;
    }

    /**
     * Updates the limelight's pose with the robot yaw. This is needed for MT2 stable detection. REQUIRED.
     * @param yaw the yaw aligned with the field map in Degrees
     */
    public void updateWithPose( double yaw )
    {
        limelight3A.updateRobotOrientation(yaw);
    }


    /**
     * Returns the estimated robot pose within LL's field coordinates. 
     * More accurately, returns the nearest LL result pose using MT2 detection, requiring an accurate robot yaw update.
     * Will return an empty optional if the LL is not on, connected, no AT's are in view, or the nearest AT is > 2m away.
     * 
     * @return an Optional of the Point representing the estimated robot pose, in LL field coordinates.
     *  Empty optional of no detected or the result is beyond 2m.
     */
    public Optional<Point> getRobotPose()
    {
        LLResult result = limelight3A.getLatestResult();
        Log.i("LIMELIGHT", "GOT POSE");
        if( result == null || !isValidResult(result) ) // 2 meters max dist
            return Optional.empty();
        Log.i("LIMELIGHT", "USING LL");
       return Optional.of(new Point(result.getBotpose_MT2().getPosition().x, result.getBotpose_MT2().getPosition().y));
    }

    private boolean isValidResult( final LLResult result )
    {
        Log.i("LIMELIGHT_STDEV_X", String.valueOf(result.getStddevMt2()[0]));
        Log.i("LIMELIGHT_STDEV_Y", String.valueOf(result.getStddevMt2()[1]));
        return result.isValid() &&
                Math.abs(result.getStddevMt2()[0]) < 0.1 && Math.abs(result.getStddevMt2()[1]) < 0.1;
    }

}
