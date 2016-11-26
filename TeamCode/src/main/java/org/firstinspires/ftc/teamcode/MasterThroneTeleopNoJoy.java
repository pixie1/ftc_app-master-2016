        package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        /**
         * Created by Karine on 10/27/2015.
         */
        @TeleOp
        public class MasterThroneTeleopNoJoy extends OpMode {
            DcMotor motorFrontRight;
            DcMotor motorFrontLeft;
            DcMotor motorBackRight;
            DcMotor motorBackLeft;

            public MasterThroneTeleopNoJoy() {
            }

            @Override
            public void init() {
                //this is where we define all of the motors on the robot.
                motorFrontRight = hardwareMap.dcMotor.get("motor_1");
                motorBackRight = hardwareMap.dcMotor.get("motor_2");
                motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
                motorBackLeft = hardwareMap.dcMotor.get("motor_4");
            }

            @Override
            public void loop() {

                //CONTROL METHOD #1: Buttons
                if (gamepad1.y) {
                    motorFrontRight.setPower(1);
                    motorBackRight.setPower(1);
                } else if (gamepad1.dpad_up) {
                    motorFrontLeft.setPower(1);
                    motorBackLeft.setPower(1);
                } else if (gamepad1.a) {
                    motorFrontRight.setPower(-1);
                    motorBackRight.setPower(-1);
                } else if (gamepad1.dpad_down) {
                    motorFrontLeft.setPower(-1);
                    motorBackLeft.setPower(-1);
                } else {
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackRight.setPower(0);
                }
            }
        }