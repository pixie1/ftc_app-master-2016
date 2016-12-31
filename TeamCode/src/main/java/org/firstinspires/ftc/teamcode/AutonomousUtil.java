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
    public AutonomousUtil(Servo catcher, Servo buttonbash, DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, Telemetry telemetry, ModernRoboticsI2cGyro sensorMRGyro) {
        this.motorBackLeft=motorBackLeft;
        this.motorBackRight=motorBackRight;
        this.motorFrontLeft=motorFrontLeft;
        this.motorFrontRight=motorFrontRight;
        this.buttonbash=buttonbash;
        this.catcher=catcher;
        this.telemetry=telemetry;
        this.sensorGyro=sensorMRGyro;
        encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro);
    }
    public void parkOnCenter(boolean red){
        buttonbash.setPosition(1);
        catcher.setPosition(0);
        ElapsedTime delay = new ElapsedTime();
        delay.reset();
           while(delay.time()<10){}
        //Robot length: 26cm
        encoderMoveUtil.forward(100 - RBL, 0.25); //position robot
        if(red=true) {
            encoderMoveUtil.turnGyro(-GYROCENTER, 0.25); //aim at ball
        } else {
            encoderMoveUtil.turnGyro(GYROCENTER,0.25);
        }
        encoderMoveUtil.forward(60, 0.50);
    }
    public void hitBall(boolean red){
        buttonbash.setPosition(1);
        catcher.setPosition(0);
        encoderMoveUtil.forward(100 - RBL, 0.25); //position robot
        if(red=true) {
            encoderMoveUtil.turnGyro(-GYROCENTER, 0.25);
        } else {
            encoderMoveUtil.turnGyro(GYROCENTER, 0.25);
        }
        encoderMoveUtil.forward(60, 0.50); //hit ball
    }
    public void ramp(boolean red){
        encoderMoveUtil.forward(40-RBL,0.25); //position robot
        if (red=true) {
            encoderMoveUtil.turnGyro(-GYRORAMP,0.25); //turn parrallel to ramp
            encoderMoveUtil.forward(70,0.25); //position near center of ramp
            encoderMoveUtil.turnGyro(-(GYRORAMP*2),0.25); //face ramp
        } else {
            encoderMoveUtil.turnGyro(GYRORAMP,0.25); //turn parrallel to ramp
            encoderMoveUtil.forward(70,0.25); //position near center of ramp
            encoderMoveUtil.turnGyro((GYRORAMP*2),0.25); //face ramp
        }
        encoderMoveUtil.forward(60,0.3); //get on ramp
    }
    public void launch(boolean red) {
        encoderMoveUtil.forward(80-RBL,0.25);
        if(red=true) {
            encoderMoveUtil.turnGyro(-GYROLAUNCH,0.25);
            encoderMoveUtil.forward(80,0.25);
            encoderMoveUtil.turnGyro(-(GYROLAUNCH*2),0.25);
        } else {
            encoderMoveUtil.turnGyro(GYROLAUNCH,0.25);
            encoderMoveUtil.forward(80,0.25);
            encoderMoveUtil.turnGyro((GYROLAUNCH*2),0.25);
        }
    }
}

