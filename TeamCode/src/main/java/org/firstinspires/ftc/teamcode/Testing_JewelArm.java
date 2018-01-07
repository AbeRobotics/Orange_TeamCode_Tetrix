package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by akanksha.joshi on 24-Dec-2017.
 */
@Autonomous(name="Jewel Arm Test", group="Autonomous")
public class Testing_JewelArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OPModeConstants opModeConstants = OPModeConstants.getInstance();
        Task_JewelArm lower = new Task_JewelArm(hardwareMap, OPModeConstants.jewelKickerArmPosition.ACTION);
        lower.Init();
        lower.PerformTask(telemetry, getRuntime());

        sleep(2000);

        Task_JewelArm raise = new Task_JewelArm(hardwareMap, OPModeConstants.jewelKickerArmPosition.REST);
        raise.Init();
        raise.PerformTask(telemetry, getRuntime());

        waitForStart();



    }
}
