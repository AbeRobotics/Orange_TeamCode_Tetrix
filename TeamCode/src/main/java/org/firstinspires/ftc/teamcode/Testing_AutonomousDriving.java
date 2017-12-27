package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by harsh.joshi on 25-Dec-2017.
 */
@Autonomous(name="Testing Auto Drive", group="Autonomous")
public class Testing_AutonomousDriving extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        OPModeConstants opModeConstants = OPModeConstants.getInstance();
        //set default speed
        opModeConstants.setAutoSpeed(OPModeConstants.AutonomousSpeed.HIGH);
        OPModeDriveHelper driver = OPModeDriveHelper.getInstance();
        driver.Init(telemetry,hardwareMap);
        waitForStart();
        //boolean taskCompleted = driver.MoveForward(24.0d, OPModeConstants.AutonomousSpeed.MEDIUM);
       //sleep(2000);
        //taskCompleted = driver.MoveBackward(24.0);
        //sleep(2000);
        driver.TurnRight();
        sleep(2000);
        driver.TurnLeft();
        sleep(2000);
        driver.TurnAround();
        sleep(2000);
        driver.Turn(45, OPModeConstants.AutonomousSpeed.MEDIUM);
    }
}
