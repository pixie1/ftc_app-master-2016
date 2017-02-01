package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    Servo catcherL;
    Servo catcherR;
    Servo buttonbashL;
    Servo buttonbashR;

    ModernRoboticsI2cGyro sensorGyro;
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor;
    OpticalDistanceSensor lightSensor;
    public Telemetry telemetry;

    //motorFrontRight = hardwareMap.dcMotor.get("motor_1");
    //motorBackRight = hardwareMap.dcMotor.get("motor_2");
    //motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
    //motorBackLeft = hardwareMap.dcMotor.get("motor_4");
    //sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

    //catcherL = hardwareMap.servo.get("servo_1");
    //buttonbashL = hardwareMap.servo.get("servo_2");


    EncoderMoveUtil encoderMoveUtil;
    final int RBL = 26;
    final int GYROCENTER = 30; //The gyro degree constant for MasterThroneAutoBall and MAsterParkOnCEnterAuto
    final int GYRORAMP = 40; //Gyro constant for MasterThroneAutoRamp
    final int GYROLAUNCH = 40;

    public AutonomousUtil(Servo catcherL, Servo buttonbashL, DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, Telemetry telemetry, ModernRoboticsI2cGyro sensorMRGyro, DcMotor launchR, DcMotor launchL, Servo buttonbashR, Servo catcherR, ModernRoboticsI2cRangeSensor rangeSensor, ColorSensor colorSensor, OpticalDistanceSensor lightSensor) {
        this.motorBackLeft = motorBackLeft;
        this.motorBackRight = motorBackRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorFrontRight = motorFrontRight;
        this.buttonbashL = buttonbashL;
        this.catcherL = catcherL;
        this.telemetry = telemetry;
        this.sensorGyro = sensorMRGyro;
        this.launchR = launchR;
        this.launchL = launchL;
        this.buttonbashR = buttonbashR;
        this.catcherR = catcherR;
        this.rangeSensor = rangeSensor;
        this.colorSensor = colorSensor;
        this.lightSensor = lightSensor;
        encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro);
    }

    public void parkOnCenter(boolean red) {
        //buttonbashL.setPosition(1);
        catcherL.setPosition(0.5);
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
        //buttonbashL.setPosition(1);
        catcherL.setPosition(0.5);
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
        encoderMoveUtil.backward(65, 0.5);
        lineLookTime.reset();
        while (lineLookTime.seconds() < 6) {
            launchL.setPower(1);
            launchR.setPower(-1);
            while (lineLookTime.seconds() < 2) {
            }
            catcherL.setPosition(1);
            catcherR.setPosition(0);
        }
        catcherL.setPosition(0.5);
        catcherR.setPosition(0.5);
        buttonbashL.setPosition(0.9);
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
            catcherL.setPosition(1);
            catcherR.setPosition(0);
        }
        catcherL.setPosition(0.5);
        catcherR.setPosition(0.5);
        encoderMoveUtil.backward(50, 0.5);
    }
    public void beaconRampEnd(boolean red) {
        int color;
        if (red) {
            color=1;
        } else {
            color = -1;
        }
        encoderMoveUtil.forward(25,0.2);
        buttonbashL.setPosition(1);
        buttonbashR.setPosition(0.3);
        encoderMoveUtil.forward(20,1);
        encoderMoveUtil.turnGyro(-45*color,.25);
        encoderMoveUtil.backward(100,1);
    }
    public void beaconBallEnd(boolean red) {
        int color;
        if (red) {
            color=1;
        } else {
            color = -1;
        }
        encoderMoveUtil.forward(40,1);
        encoderMoveUtil.turnGyro(120*color,.25);
        encoderMoveUtil.backward(80,1);
        encoderMoveUtil.turnGyro(90*color,.25);
        encoderMoveUtil.backward(60,1);
    }
    public void beaconLauncher (boolean red) {
        ElapsedTime lineLookTime = new ElapsedTime();
        encoderMoveUtil.backward(60, 0.3);
        //launch particle code
        lineLookTime.reset();
        launchL.setPower(1);
        launchR.setPower(-1);
        while(lineLookTime.seconds()<= .5){}
        catcherL.setPosition(.8);
        catcherR.setPosition(.2);
        while(lineLookTime.seconds()<= 4){}
        launchL.setPower(0);
        launchR.setPower(0);
        catcherL.setPosition(.5);
        catcherR.setPosition(.5);
    }
    public void beacon(boolean red) {
        int gyroturn2;
        ElapsedTime lineLookTime = new ElapsedTime();
        lineLookTime.reset();
        int color;
        if (red) {
            color=1;
        } else {
            color = -1;
        }
        encoderMoveUtil.turnGyro(-90*color, 0.2);
        encoderMoveUtil.backward(55, 0.5);
        if (red) {
            gyroturn2 = 45 + sensorGyro.getIntegratedZValue();
        }else {
            gyroturn2 = 45 - sensorGyro.getIntegratedZValue();
        }
            telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyro(gyroturn2,0.2); //TURNING TOWARDS LINE
        lineLookTime.reset();
        //LOOKING FOR LINE
        while (lightSensor.getLightDetected() < 0.7 && lineLookTime.seconds() < 5) {
            telemetry.addData("LIGHT", lightSensor.getLightDetected());
            telemetry.update();
            motorBackLeft.setPower(0.15);
            motorBackRight.setPower(0.15);
            motorFrontLeft.setPower(0.15);
            motorFrontRight.setPower(0.15);
        }
        // Line detected, moving backward and turning to beacon
        lineLookTime.reset();
        telemetry.addData("LINE FOUND", 0);
        encoderMoveUtil.forward(10,0.2);
        encoderMoveUtil.stopMotors();
        buttonbashL.setPosition(1);
        buttonbashR.setPosition(0.3);
        if (red) {
            gyroturn2 = 90 + sensorGyro.getIntegratedZValue();
        }else {
            gyroturn2 = 90 - sensorGyro.getIntegratedZValue();
        }
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyro(gyroturn2,0.2);//TURNING AND FACING BEACON
        if (red) {
            gyroturn2 = 90 + sensorGyro.getIntegratedZValue();
        }else {
            gyroturn2 = 90 - sensorGyro.getIntegratedZValue();
        }
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyroPrecise(gyroturn2,0.160);
        while (rangeSensor.cmUltrasonic() < 100 && rangeSensor.cmUltrasonic() > 20) { //Run up to beacon
            motorFrontLeft.setPower(0.15);
            motorFrontRight.setPower(0.15);
            motorBackLeft.setPower(0.15);
            motorBackRight.setPower(0.15);
            telemetry.addData("DIST:", rangeSensor.cmUltrasonic());
            telemetry.update();
        }
        telemetry.addData("BEACON REACHED","");
        buttonbashR.setPosition(0.7);
        encoderMoveUtil.stopMotors();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        encoderMoveUtil.backward(5,0.15);//Move a bit forward
        if (red) { //Based on autonomous, look for the correct colour and hit the right button
            if (colorSensor.red() >= 1) {
                telemetry.addData("RED", "");
                buttonbashR.setPosition(0.8);
                buttonbashL.setPosition(1);
            }
            if (colorSensor.blue() >= 1) {
                telemetry.addData("BLUE", "");
                buttonbashL.setPosition(0);
                buttonbashR.setPosition(0.2);
            }
        } else {
            if (colorSensor.red() >= 1) {
                telemetry.addData("RED", "");
                buttonbashR.setPosition(0.2);
                buttonbashL.setPosition(0);
            }
            if (colorSensor.blue() >= 1) {
                telemetry.addData("BLUE", "");
                buttonbashL.setPosition(1);
                buttonbashR.setPosition(0.8);
            }
        }
        encoderMoveUtil.forward(5,0.15);
        encoderMoveUtil.backward(11,0.15); // Beacon has been hit, waiting then stopping
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
