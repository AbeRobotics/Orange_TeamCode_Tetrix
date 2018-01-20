package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by akanksha.joshi on 25-Dec-2017.
 */
@Autonomous(name="Testing Gyro Drive", group="Autonomous")
public class Testing_GyroDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        OPModeConstants opModeConstants = OPModeConstants.getInstance();
        OPModeDriveHelper driveHelper = OPModeDriveHelper.getInstance();
        driveHelper.Init(telemetry, hardwareMap);

        waitForStart();
        driveHelper.gyroTurn(0.3,45);
        sleep(1000);
        driveHelper.gyroTurn(0.3,-90);
        sleep(1000);


        /* driveHelper.gyroDrive(0.5,-45,0);
        sleep(1000);
        driveHelper.gyroTurn(0.3,-90);
        sleep(1000);
        driveHelper.gyroDrive(0.5,-12,0);
        sleep(1000);
        driveHelper.gyroTurn(0.3,90);
        sleep(1000);
        driveHelper.gyroDrive(0.5,-12,0);*/
        /*DcMotor leftWheel = hardwareMap.dcMotor.get("left_wheel");
        DcMotor rightWheel = hardwareMap.dcMotor.get("right_wheel");
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftWheel.setPower(1);
        rightWheel.setPower(1);
        sleep(2000);
        leftWheel.setPower(0);
        rightWheel.setPower(0);*/

    }
}
