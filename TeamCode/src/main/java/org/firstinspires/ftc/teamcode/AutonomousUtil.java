package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Karine on 12/27/2016.
 */
public class AutonomousUtil {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchL;
    DcMotor launchR;


    Servo catcher;
    Servo buttonbash;
    Servo buttonbashR;
    ModernRoboticsI2cGyro sensorGyro;
    public Telemetry telemetry;

    //motorFrontRight = hardwareMap.dcMotor.get("motor_1");
    //motorBackRight = hardwareMap.dcMotor.get("motor_2");
    //motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
    //motorBackLeft = hardwareMap.dcMotor.get("motor_4");
    //sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

    //catcher = hardwareMap.servo.get("servo_1");
    //buttonbash = hardwareMap.servo.get("servo_2");


    EncoderMoveUtil encoderMoveUtil;
    final int RBL = 26;
    final int GYROCENTER = 30; //The gyro degree constant for MasterThroneAutoBall and MAsterParkOnCEnterAuto
    final int GYRORAMP = 40; //Gyro constant for MasterThroneAutoRamp
    final int GYROLAUNCH = 40;

    public AutonomousUtil(Servo catcher, Servo buttonbash, DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, Telemetry telemetry, ModernRoboticsI2cGyro sensorMRGyro, DcMotor launchR, DcMotor launchL, Servo buttonbashR) {
        this.motorBackLeft = motorBackLeft;
        this.motorBackRight = motorBackRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorFrontRight = motorFrontRight;
        this.buttonbash = buttonbash;
        this.catcher = catcher;
        this.telemetry = telemetry;
        this.sensorGyro = sensorMRGyro;
        this.launchR = launchR;
        this.launchL = launchL;
        this.buttonbashR = buttonbashR;
        encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro);
    }

    public void parkOnCenter(boolean red) {
        //buttonbash.setPosition(1);
        catcher.setPosition(0.5);
        ElapsedTime delay = new ElapsedTime();
        delay.reset();
        while (delay.time() < 10) {
        }
        //Robot length: 26cm
        encoderMoveUtil.forward(100 - RBL, 0.25); //position robot
        if (red = true) {
            encoderMoveUtil.turnGyro(-GYROCENTER, 0.25); //aim at ball
        } else {
            encoderMoveUtil.turnGyro(GYROCENTER, 0.25);
        }
        encoderMoveUtil.forward(60, 0.50);
    }

    public void hitBall(boolean red) {
        //buttonbash.setPosition(1);
        catcher.setPosition(0.5);
        encoderMoveUtil.forward(100 - RBL, 0.25); //position robot
        if (red = true) {
            encoderMoveUtil.turnGyro(-GYROCENTER, 0.25);
        } else {
            encoderMoveUtil.turnGyro(GYROCENTER, 0.25);
        }
        encoderMoveUtil.forward(60, 0.50); //hit ball
    }

    public void ramp(boolean red) {
        encoderMoveUtil.forward(40 - RBL, 0.25); //position robot
        if (red = true) {
            encoderMoveUtil.turnGyro(-GYRORAMP, 0.25); //turn parrallel to ramp
            encoderMoveUtil.forward(70, 0.25); //position near center of ramp
            encoderMoveUtil.turnGyro(-(GYRORAMP * 2), 0.25); //face ramp
        } else {
            encoderMoveUtil.turnGyro(GYRORAMP, 0.25); //turn parrallel to ramp
            encoderMoveUtil.forward(70, 0.25); //position near center of ramp
            encoderMoveUtil.turnGyro((GYRORAMP * 2), 0.25); //face ramp
        }
        encoderMoveUtil.forward(60, 0.3); //get on ramp
    }

    public void launch(boolean red) {
        int color;
        if (red) {
            color=1;
        } else {
            color=-1;
        }
        ElapsedTime lineLookTime = new ElapsedTime();
        encoderMoveUtil.backward(75, 0.5);
        lineLookTime.reset();
        while (lineLookTime.seconds() < 6) {
            launchL.setPower(1);
            launchR.setPower(-1);
            while (lineLookTime.seconds() < 2) {
            }
            catcher.setPosition(1);
        }
        catcher.setPosition(0.5);
        buttonbash.setPosition(0.9);
        buttonbashR.setPosition(0.2);
        encoderMoveUtil.turnGyro(30*color, 0.2);
        encoderMoveUtil.backward(25, 0.5);
        encoderMoveUtil.turnGyro(-45*color, 0.2);
        encoderMoveUtil.backward(40, 0.5);
    }

    public void launchUpper(boolean red) {
        ElapsedTime lineLookTime = new ElapsedTime();
        int color;
        if (red) {
            color=1;
        } else {
            color=-1;
        }
        encoderMoveUtil.backward(55, 0.5);
        encoderMoveUtil.turnGyroPrecise(-35*color, 0.2);
        encoderMoveUtil.backward(35, 0.5);
        lineLookTime.reset();
        while (lineLookTime.seconds() < 6) {
            launchL.setPower(1);
            launchR.setPower(-1);
            while (lineLookTime.seconds() < 2) {
            }
            catcher.setPosition(1);
        }
        catcher.setPosition(0.5);
        encoderMoveUtil.backward(50, 0.5);
    }
}
