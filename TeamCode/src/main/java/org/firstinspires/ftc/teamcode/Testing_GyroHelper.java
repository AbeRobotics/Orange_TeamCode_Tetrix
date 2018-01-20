package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by akanksha.joshi on 25-Dec-2017.
 */
@Autonomous(name="Testing Gyro Helper", group="Autonomous")
public class Testing_GyroHelper extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        OPModeConstants opModeConstants = OPModeConstants.getInstance();
        OPModeGyroHelper gyroHelper = OPModeGyroHelper.getInstance();
        gyroHelper.Init(telemetry, hardwareMap);
        waitForStart();
        double heading = 0.0d;

        while(opModeIsActive()){
            heading = gyroHelper.GetGyroAngle();
            telemetry.addData("Heading: ", heading);
            telemetry.update();
        }
        gyroHelper.Reset();

    }
}
