package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpMode_MainCar", group="Iterative Opmode")

public class TeleOpMode_MainCar extends OpMode
{
    
    //Main Timer
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //DcMotors
    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor leftChain;
    private DcMotor rightChain;
    private DcMotor arm_1;
    private DcMotor arm_2;

    //Servos
    private Servo f_clawServo;
    private Servo b_clawServo;
    private Servo arm_servo_1;
    private Servo arm_servo_2;


    //wheel_servos
    private CRServo right_wheel;
    private CRServo left_wheel;

    public int Chain_exp = 0;

    @Override
    public void init() {
        //Print
        telemetry.addData("Status", "init() Running");
        telemetry.update();
        //initiating motors (Normal)
        leftfront  = hardwareMap.get(DcMotor.class, "l_f");
        rightfront = hardwareMap.get(DcMotor.class, "r_f");
        leftrear = hardwareMap.get(DcMotor.class, "l_b");
        rightrear = hardwareMap.get(DcMotor.class,"r_b");
        leftChain = hardwareMap.get(DcMotor.class,"leftchain");
        rightChain = hardwareMap.get(DcMotor.class,"rightchain");
        arm_1 = hardwareMap.get(DcMotor.class,"arm_1");
        arm_2 = hardwareMap.get(DcMotor.class,"arm_2");

        //initiating Servos (Normal)
        f_clawServo = hardwareMap.get(Servo.class,"fclaw");
        b_clawServo = hardwareMap.get(Servo.class,"bclaw");
        arm_servo_1 = hardwareMap.get(Servo.class,"armservo1");
        arm_servo_2 = hardwareMap.get(Servo.class,"armservo2");


        //init CRServos
        right_wheel = hardwareMap.get(CRServo.class,"rwheel");
        left_wheel = hardwareMap.get(CRServo.class,"lwheel");

        //set motor direction
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.REVERSE);
        leftChain.setDirection(DcMotor.Direction.FORWARD);
        rightChain.setDirection(DcMotor.Direction.REVERSE);
        right_wheel.setDirection(CRServo.Direction.FORWARD);
        left_wheel.setDirection(CRServo.Direction.REVERSE);
        arm_1.setDirection(CRServo.Direction.FORWARD);
        arm_2.setDirection(CRServo.Direction.REVERSE);


        //config encoder run modes
        leftChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Config mode on start
        leftChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        f_clawServo.scaleRange(0.4,0.7);
        b_clawServo.scaleRange(0.4,0.7);
        telemetry.addData("F_claw POS",f_clawServo.getPosition());
        telemetry.addData("B_claw POS",b_clawServo.getPosition());

        right_wheel.setPower(0);
        left_wheel.setPower(0);

        //Print
        telemetry.addData("Status", "init() Done");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        timer.reset();
    }

    @Override
    public void start() {
        timer.reset();
        telemetry.addData("Timer", "%.3f",timer.time());
        telemetry.update();
    }

    @Override
    public void loop()   {
        //Encoder position
        int leftChain_Pos = leftChain.getCurrentPosition();
        int rightChain_Pos = rightChain.getCurrentPosition();
        telemetry.addData("Left_Pos:",leftChain_Pos);
        telemetry.addData("Right_Pos:",rightChain_Pos);
        telemetry.addData("Exp_Pos:",Chain_exp);

        //motor power settings
        double power_1;
        double power_2;
        double power_3;
        double power_4;
        //raw stick input (Reverse both Y axis)
        double gamepad1_X = gamepad1.left_stick_x; //leftX
        double gamepad1_Y = -gamepad1.left_stick_y; //leftY
        double gamepad1_Z = gamepad1.right_stick_x; //RightX
        double gamepad1_W = -gamepad1.right_stick_y; //RightY
        boolean gamepad1_a = gamepad1.a;
        boolean gamepad1_b = gamepad1.b;
        boolean gamepad1_x = gamepad1.x;
        boolean gamepad1_y = gamepad1.y;
        double f_gamepad1_servo = gamepad1.right_trigger;
        double b_gamepad1_servo = gamepad1.left_trigger;
        boolean gamepad1_arm_l = gamepad1.left_bumper;
        boolean gamepad1_arm_r = gamepad1.right_bumper;
        boolean gamepad1_arm_servo1_u = gamepad1.dpad_up;
        boolean gamepad1_arm_servo1_d = gamepad1.dpad_down;
        boolean gamepad1_arm_servo2_l = gamepad1.dpad_left;
        boolean gamepad1_arm_servo2_r = gamepad1.dpad_right;

        //power raw
        double dChasisScale = Math.pow(Math.hypot(gamepad1_X,gamepad1_Y),2);

        power_1 =Functions.MecDrive_RightFront(
                dChasisScale*Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                dChasisScale*Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
                );
        power_2 = Functions.MecDrive_LeftFront(
                dChasisScale*Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                dChasisScale*Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        power_3 = Functions.MecDrive_LeftRear(
                dChasisScale*Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                dChasisScale*Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        power_4 = Functions.MecDrive_RightRear(
                dChasisScale*Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                dChasisScale*Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        //scale output so that power is scaled with maximum of 1
        double trim_max = Math.max(Math.max(Math.max(Math.abs(power_1),Math.abs(power_2)),
                Math.max(Math.abs(power_3),Math.abs(power_4))),1);
        power_1 /= trim_max;
        power_2 /= trim_max;
        power_3 /= trim_max;
        power_4 /= trim_max;

        //set motor power
        leftfront.setPower(power_2);
        leftrear.setPower(power_3);
        rightfront.setPower(power_1);
        rightrear.setPower(power_4);

        //chain mapped on button A&B on gamepad1
        double dChainSpeed = 0.3;

        if (Math.abs(leftChain_Pos-rightChain_Pos)>50) {
            leftChain.setTargetPosition((leftChain_Pos+rightChain_Pos)/2);
            leftChain.setPower(dChainSpeed);
            rightChain.setTargetPosition((leftChain_Pos+rightChain_Pos)/2);
            rightChain.setPower(dChainSpeed);
        }
        else {
            if (gamepad1_a) {
                Chain_exp = leftChain_Pos + 100;
                leftChain.setTargetPosition(Chain_exp);
                leftChain.setPower(dChainSpeed);
                rightChain.setTargetPosition(Chain_exp);
                rightChain.setPower(dChainSpeed);

            }
            else if (gamepad1_b) {
                Chain_exp = leftChain_Pos - 100;
                leftChain.setTargetPosition(Chain_exp);
                leftChain.setPower(dChainSpeed);
                rightChain.setTargetPosition(Chain_exp);
                rightChain.setPower(dChainSpeed);
            }
            else {
                leftChain.setTargetPosition(Chain_exp);
                leftChain.setPower(dChainSpeed);
                rightChain.setTargetPosition(Chain_exp);
                rightChain.setPower(dChainSpeed);
            }
        }

        //CRServos

        if (gamepad1_x) {
            left_wheel.setPower(1);
            right_wheel.setPower(1);
        }
        else if (gamepad1_y) {
            left_wheel.setPower(-1);
            right_wheel.setPower(-1);
        }
        else {
            left_wheel.setPower(0);
            right_wheel.setPower(0);
        }

        //Claw Servos

        f_clawServo.setPosition(f_gamepad1_servo);
        b_clawServo.setPosition(-b_gamepad1_servo);

        telemetry.addData("f_claw_pos:",f_clawServo.getPosition());
        telemetry.addData("b_claw_pos:",b_clawServo.getPosition());

        //Long Arm
        if (gamepad1_arm_l) {
            arm_1.setPower(0.8);
            arm_2.setPower(0.8);
        }
        else if (gamepad1_arm_r) {
            arm_1.setPower(-0.8);
            arm_2.setPower(-0.8);
        }
        else {
            arm_1.setPower(0);
            arm_2.setPower(0);
        }


        //put data into dashboard
        telemetry.addData("Config_Main_Timer", "Run Time: " + timer.toString());
        telemetry.addData("Config_trim",trim_max);
        //telemetry.addData("gamepad1.atRest()",gamepad1.atRest());
        telemetry.addData("Config_gamepad1.Input","LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)",
                gamepad1_X,gamepad1_Y,gamepad1_Z,gamepad1_W);
        telemetry.addData("Config_MotorPower","1 (%.2f), 2 (%.2f), 3 (%.2f),4 (%.2f)",
                power_1,power_2,power_3,power_4);

    }

    @Override
    public void stop() {
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
        leftChain.setPower(0);
        rightChain.setPower(0);
        left_wheel.setPower(0);
        right_wheel.setPower(0);
        telemetry.clearAll();
        telemetry.addData("Status","Stop Enforced");
        telemetry.update();
    }

}