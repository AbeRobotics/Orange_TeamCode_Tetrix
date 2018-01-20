package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Akanksha.Joshi on 25-Dec-2017.
 */

public class OPModeDriveHelper {
    private static OPModeDriveHelper instance;
    private  OPModeConstants opModeConstants;
    private  Telemetry telemetry;
    private  HardwareMap hardwareMap;
    private OPModeGyroHelper gyroHelper;
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
        gyroHelper = OPModeGyroHelper.getInstance();
        gyroHelper.Init(telemetry,hardwareMap);

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

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setPower(GetPower(speed));
        rightWheel.setPower(GetPower(speed));
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
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyroHelper.GetGyroAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= OPModeConstants.gyroThreshold) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            rightSpeed*=-1;
            leftSpeed = rightSpeed;
        }

        // Send desired speeds to motors.
        leftWheel.setPower(leftSpeed);
        rightWheel.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void gyroTurn (  double speed, double angle) {

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.


        while ( !onHeading(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        SetAllStop();
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