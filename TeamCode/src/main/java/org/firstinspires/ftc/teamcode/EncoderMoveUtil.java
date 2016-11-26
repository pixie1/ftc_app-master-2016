package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.TelemetryImpl;

/**
 * Created by Karine on 2/13/2016.
 */
public class EncoderMoveUtil {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    ModernRoboticsI2cGyro sensorGyro;
    public Telemetry telemetry;
    int counter;
    boolean correctPos;

    public EncoderMoveUtil(DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, Telemetry telemetry){
        this.motorBackLeft=motorBackLeft;
        this.motorBackRight=motorBackRight;
        this.motorFrontLeft=motorFrontLeft;
        this.motorFrontRight=motorFrontRight;
        this.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry=telemetry;
    }

    public EncoderMoveUtil(DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, ModernRoboticsI2cGyro sensorGyro){
        this.sensorGyro=sensorGyro;
        this.motorBackLeft=motorBackLeft;
        this.motorBackRight=motorBackRight;
        this.motorFrontLeft=motorFrontLeft;
        this.motorFrontRight=motorFrontRight;
        this.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int cmToEncoderTicks(double cm) {
        double d = 2.54 * 5;
        double pi = 3.1415;
        double encoderConstant = 280; //NEED TO CHANGE FOR ANDYMARK
        double rotationConstant = d * pi;
        Double doubleEncoderTicks = (cm * (1 / rotationConstant)) * encoderConstant;
        int encoderTicks = doubleEncoderTicks.intValue();
        return encoderTicks;

    }

    public void stopMotors() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public  int angleToEncoderTicks(double turnAmount) {

        double s = ((turnAmount) / 360 * (2 * Math.PI)) * (17.51 / 2);
        double cir = 4 * Math.PI; //4in wheels diameter
        double numOfRotations = s / cir;
        Double encoderTicks = numOfRotations * 1440;
        int returnEncoderTicks = (encoderTicks.intValue())*2;
        return returnEncoderTicks;
    }

    public void turnCC(double turnAngle, double speed) {

        int previousPosition = motorFrontLeft.getCurrentPosition();

        int leftTicks = angleToEncoderTicks(turnAngle);
        telemetry.addData("Now turning", 5);
        telemetry.addData("leftTicks", leftTicks);
        telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
        telemetry.addData("previous Position", previousPosition);
        while (Math.abs(motorFrontLeft.getCurrentPosition()- previousPosition)< Math.abs(leftTicks)) // || motorLeft.getCurrentPosition()<rightTicks)
        {
            motorFrontRight.setPower(-speed);
            motorFrontLeft.setPower(speed);
            motorBackRight.setPower(-speed);
            motorBackLeft.setPower(speed);
            telemetry.addData("Now turning", 5);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
        }
        telemetry.addData("IZV", sensorGyro.getIntegratedZValue());
        stopMotors();
    }

    public void turnC(double turnAngle, double speed) {

        int previous = motorFrontLeft.getCurrentPosition();
        int leftTicks = angleToEncoderTicks(turnAngle);
        telemetry.addData("Now turning", 6);
        telemetry.addData("leftTicks", leftTicks);
        while (Math.abs(motorFrontLeft.getCurrentPosition()-previous) <Math.abs(leftTicks))// || motorLeft.getCurrentPosition()<rightTicks)
        {
            motorFrontRight.setPower(speed);
            motorFrontLeft.setPower(-speed);
            motorBackRight.setPower(speed);
            motorBackLeft.setPower(-speed);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
        }
        telemetry.addData("IZV", sensorGyro.getIntegratedZValue());
        stopMotors();
    }

    public void forward(double disInCm, double speed) {
        int current = motorFrontLeft.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        while (Math.abs(motorFrontLeft.getCurrentPosition()) < disInEncoderTicks + Math.abs(current)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            motorFrontLeft.setPower(-speed);
            motorFrontRight.setPower(-speed);
            motorBackLeft.setPower(-speed);
            motorBackRight.setPower(-speed);
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void backward(double disInCm, double speed) {
        int current = motorFrontLeft.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        while (Math.abs(Math.abs(motorFrontLeft.getCurrentPosition())-Math.abs(current)) < Math.abs(disInEncoderTicks)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            motorFrontLeft.setPower(speed);
            motorFrontRight.setPower(speed);
            motorBackLeft.setPower(speed);
            motorFrontRight.setPower(speed);
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);

    }

    public void checkAngleC(int targetAngle, int aZero, int threshold) {
        counter=0;
        correctPos=false;
        while(counter<=threshold&&correctPos==false) {
            int angleDifference = Math.abs(sensorGyro.getIntegratedZValue()) - (Math.abs(aZero - targetAngle));
            telemetry.addData("IZV", sensorGyro.getIntegratedZValue());
            telemetry.addData("angleDifference", angleDifference);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (angleDifference < -2) {
                turnC(angleDifference, 0.5);
                correctPos=false;
                counter++;
            } else if (angleDifference > 2) {
                turnCC(angleDifference, 0.5);
                correctPos=false;
                counter++;
            }else{
                correctPos=true;
            }
        }
    }

    public void checkAngleCC(int targetAngle, int aZero, int threshold) {
        counter = 0;
        correctPos = false;
        while (counter <= threshold && correctPos == true) {
            int angleDifference = Math.abs(sensorGyro.getIntegratedZValue()) - (Math.abs(aZero + targetAngle));
            telemetry.addData("IZV", sensorGyro.getIntegratedZValue());
            telemetry.addData("angleDifference", angleDifference);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (angleDifference < -2) {
                turnCC(angleDifference, 0.5);
                counter++;
                correctPos = false;
            } else if (angleDifference > 2) {
                turnC(angleDifference, 0.5);
                counter++;
                correctPos = false;
            } else {
                correctPos = true;
            }
        }
    }
    public void turnGyro(int targetRelativeHeading, double speed){
        int angleCurrent = sensorGyro.getIntegratedZValue();
        int targetHeading=angleCurrent+targetRelativeHeading;
        while(targetRelativeHeading>targetHeading){
            motorBackLeft.setPower(-speed);
            motorBackRight.setPower(speed);
            motorFrontLeft.setPower(-speed);
            motorFrontRight.setPower(speed);
            telemetry.addData("HeadingCurrent", angleCurrent);
            telemetry.addData("Target",targetRelativeHeading);
        }
        while(targetRelativeHeading<targetHeading){
            motorBackLeft.setPower(speed);
            motorBackRight.setPower(-speed);
            motorFrontLeft.setPower(speed);
            motorFrontRight.setPower(-speed);
            telemetry.addData("HeadingCurrent", angleCurrent);
            telemetry.addData("Target",targetRelativeHeading);
        }
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        telemetry.addData("TurnDone",0);
    }
}
