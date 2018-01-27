package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by akanksha.joshi on 25-Dec-2017.
 */
@Autonomous(name="Testing Gyro Helper V2", group="Autonomous")
public class Testing_GyroHelperV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

       OPModeGyroHelperV2 gyroHelper = new OPModeGyroHelperV2();
        waitForStart();
        double heading = 0.0d;

        while(opModeIsActive()){
            heading = gyroHelper.GetGyroAngle(telemetry,hardwareMap);
            telemetry.addData("Heading: ", heading);
            telemetry.update();
        }


    }
}
