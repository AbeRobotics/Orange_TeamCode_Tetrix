package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Akanksha.Joshi on 25-Dec-2017.
 */

public class OPModeDriveHelper {
    private static OPModeDriveHelper instance;
    private  OPModeConstants opModeConstants;
    private  Telemetry telemetry;
    private  HardwareMap hardwareMap;
    DcMotor leftWheel;
    DcMotor rightWheel;
    public static OPModeDriveHelper getInstance()
    {
        if(instance==null)
        {
            instance = new OPModeDriveHelper();

        }
        return instance;
    }
    public void Init(Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry= telemetry;
        this.hardwareMap = hardwareMap;
        opModeConstants = OPModeConstants.getInstance();

        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
    }
    private OPModeDriveHelper(){}

    public boolean MoveForward(Double inches)
    {
        return MoveForward(inches, opModeConstants.getAutoSpeed());
    }
    public boolean MoveForward(Double inches, OPModeConstants.AutonomousSpeed speed)
    {
        ResetDriveEncoders();
        SetForwardSteering();
        double totalTicks = (OPModeConstants.ticksPerInch * inches / OPModeConstants.gearRatio);
        totalTicks *=-1;
        leftWheel.setPower(GetPower(speed));
        rightWheel.setPower(GetPower(speed));
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setTargetPosition((int)totalTicks);
        rightWheel.setTargetPosition((int)(totalTicks*-1));
        while(leftWheel.isBusy())
        {
            telemetry.addData("Left Current Position -",leftWheel.getCurrentPosition());
            telemetry.addData("Right Current Position -",rightWheel.getCurrentPosition());
            telemetry.update();
            sleep(200);

        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        ResetDriveEncoders();
        return true;
    }
    public boolean MoveBackward(Double inches)
    {
        SetAllStop();
        ResetDriveEncoders();
        return MoveBackward(inches,opModeConstants.getAutoSpeed());
    }
    public boolean TurnRight()
    {
        return TurnRight(OPModeConstants.AutonomousSpeed.MEDIUM);
    }
    public boolean TurnLeft(OPModeConstants.AutonomousSpeed speed)
    {
        return Turn(-90, speed);
    }
    public boolean TurnLeft()
    {
        return TurnLeft(OPModeConstants.AutonomousSpeed.MEDIUM);
    }
    public boolean TurnAround()
    {
        return TurnAround(OPModeConstants.AutonomousSpeed.MEDIUM);
    }
    public boolean TurnAround(OPModeConstants.AutonomousSpeed speed)
    {
        return Turn(180, speed);
    }
    public boolean TurnRight(OPModeConstants.AutonomousSpeed speed)
    {
        return Turn(90, speed);
    }

    public boolean Turn(int degrees, OPModeConstants.AutonomousSpeed speed, boolean returnToOriginal){
        if(!returnToOriginal){
            return Turn(degrees, speed);
        }
        else{
            opModeConstants.setAutoSpeed(OPModeConstants.AutonomousSpeed.SLOW);
            ResetDriveEncoders();
            SetForwardSteering();
            double inchesToMove = OPModeConstants.degreesToInch * degrees;
            int ticksToMove = (int)Math.round(OPModeConstants.ticksPerInch * inchesToMove / OPModeConstants.gearRatio);
            SetClockWiseSteering();

            leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftWheel.setTargetPosition(ticksToMove);
            rightWheel.setTargetPosition(ticksToMove);

            leftWheel.setPower(GetPower(speed));
            rightWheel.setPower(GetPower(speed));

            int actionCounter = 0;
            while(leftWheel.isBusy() && actionCounter < 15)
            {
                actionCounter++;
                telemetry.addData("Left Current Position -",leftWheel.getCurrentPosition());
                telemetry.addData("Right Current Position -",rightWheel.getCurrentPosition());
                telemetry.update();
                sleep(200);


            }
            SetAllStop();
            ////////////Go back to start


            //////////////////////////Temp Hack to raise arm///////////////////////
            Task_JewelArm jewelArm = new Task_JewelArm(hardwareMap, OPModeConstants.jewelKickerArmPosition.REST);
            jewelArm.Init();
            jewelArm.PerformTask(telemetry,0);
            jewelArm.Reset();
            //////////////////////////////End//////////////////////////////////////
            leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftWheel.setTargetPosition(0);
            rightWheel.setTargetPosition(0);

            leftWheel.setPower(GetPower(speed));
            rightWheel.setPower(GetPower(speed));


             actionCounter = 0;
            while(leftWheel.isBusy() && actionCounter < 15)
            {
                actionCounter++;
                telemetry.addData("Left Current Position -",leftWheel.getCurrentPosition());
                telemetry.addData("Right Current Position -",rightWheel.getCurrentPosition());
                telemetry.update();
                sleep(200);


            }
            SetAllStop();
            ResetDriveEncoders();
            return true;
        }
    }
    public boolean Turn(int degrees, OPModeConstants.AutonomousSpeed speed)
    {
        opModeConstants.setAutoSpeed(OPModeConstants.AutonomousSpeed.SLOW);
        ResetDriveEncoders();
        SetForwardSteering();
        double inchesToMove = OPModeConstants.degreesToInch * degrees;
        int ticksToMove = (int)Math.round(OPModeConstants.ticksPerInch * inchesToMove / OPModeConstants.gearRatio);
        SetClockWiseSteering();

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(GetPower(speed));
        rightWheel.setPower(GetPower(speed));

        leftWheel.setTargetPosition(ticksToMove);
        rightWheel.setTargetPosition(ticksToMove);
        int actionCounter = 0;
        while(leftWheel.isBusy() && actionCounter < 25)
        {
            actionCounter++;
            telemetry.addData("Left Current Position -",leftWheel.getCurrentPosition());
            telemetry.addData("Right Current Position -",rightWheel.getCurrentPosition());
            telemetry.update();
            sleep(200);


        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        ResetDriveEncoders();
        return true;
    }
    public boolean MoveBackward(Double inches, OPModeConstants.AutonomousSpeed speed)
    {
        ResetDriveEncoders();
        SetReverseSteering();
        double totalTicks = OPModeConstants.ticksPerInch * inches / OPModeConstants.gearRatio;

        leftWheel.setPower(GetPower(speed));
        rightWheel.setPower(GetPower(speed));
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setTargetPosition((int)totalTicks);
        rightWheel.setTargetPosition((int)(totalTicks*-1));
        while(leftWheel.isBusy())
        {
            sleep(200);

        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        ResetDriveEncoders();
        return true;
    }
    public void SetClockWiseSteering()
    {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }
    public void SetCounterClockWiseSteering()
    {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }
    public void SetForwardSteering()
    {
        leftWheel.setPower(0);
        rightWheel.setPower(0);


    }
    public void SetReverseSteering()
    {
        leftWheel.setPower(0);
        rightWheel.setPower(0);


    }
    public void ResetDriveEncoders()
    {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double GetPower(OPModeConstants.AutonomousSpeed speed)
    {
        if(speed == OPModeConstants.AutonomousSpeed.HIGH)
            return 1.0;
        if(speed == OPModeConstants.AutonomousSpeed.MEDIUM)
            return 0.75;
        if(speed == OPModeConstants.AutonomousSpeed.SLOW)
            return 0.5;

        return 0.25;

    }
    public void SetAllStop(){
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
