package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousMode_StringCar_Test", group="Linear Opmode")

public class AutonomousMode_StringCar_Test extends LinearOpMode {

    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private ModernRoboticsI2cGyro gyro;

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private final boolean useGyro = true;

    @Override
    public void runOpMode() {
        /***Hardware initiate***/

        //initiating motors (Normal)
        leftfront  = hardwareMap.get(DcMotor.class, "motor2");
        rightfront = hardwareMap.get(DcMotor.class, "motor1");
        leftrear = hardwareMap.get(DcMotor.class, "motor3");
        rightrear = hardwareMap.get(DcMotor.class,"motor4");
        leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");

        //set motor direction
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //calibrate gyro
        if(useGyro){
            gyro = hardwareMap.get(ModernRoboticsI2cGyro.class,"gyro");
            telemetry.addData("Gyro_Stat","Caliberating...");
            telemetry.update();
            gyro.calibrate();
            while(gyro.isCalibrating()&&!isStopRequested()){
                sleep(50);
                idle();
            }
            telemetry.addData("Gyro_Stat","^_^");
            telemetry.update();
        }

        //set motor mode
        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for start todo marker
        waitForStart();

        //reset after start
        timer.reset();
        gyro.resetZAxisIntegrator();

        sleep(3000);
        analogRun(0.2,0.2,0,1);
        //autoTurning(-90,5);
        //autoTurning(90,5);


        while (opModeIsActive()) {
            idle();
        }

    }

    /**
     * Runable Speed Time analog run
     */
    private void analogRun(double analog_x, double analog_y, double analog_z, double timeOut){
        telemetry.addData("analogRun","isMoving");
        ElapsedTime turnTimeoutTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(opModeIsActive()&&turnTimeoutTimer.time()<=timeOut) {
            double power_1;
            double power_2;
            double power_3;
            double power_4;
            double trim_max;
            double gamepad1_X = -analog_x; //leftX
            double gamepad1_Y = analog_y; //leftY
            double gamepad1_Z = analog_z; //RightX
            double gamepad1_W = 0;//not used in analog

            power_1 = Functions.MecDrive_RightFront(
                    Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
            );
            power_2 = Functions.MecDrive_LeftFront(
                    Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
            );
            power_3 = Functions.MecDrive_LeftRear(
                    Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
            );
            power_4 = Functions.MecDrive_RightRear(
                    Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                    Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
            );

            //scale output so that power is scaled with maximum of 1
            trim_max = Math.max(Math.max(Math.max(Math.abs(power_1),Math.abs(power_2)),
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

            telemetry.update();
        }
        telemetry.addData("analogRun","Completed");



    }


    /**
     * Runnable Turning
     * */
    private void autoTurning(double targetAngle, double timeOut){
        telemetry.addData("autoTurning","isMoving");
        ElapsedTime turnTimeoutTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(opModeIsActive()&&(turnTimeoutTimer.time()<=timeOut&&!isLocked(targetAngle))) {
            telemetry.update();
        }
        telemetry.addData("autoTurning","Completed");
    }
    private void autoTurning(double targetAngle){
        telemetry.addData("autoTurning","isMoving");
        ElapsedTime turnTimeoutTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(opModeIsActive()&&!isLocked(targetAngle)) {
            telemetry.update();
        }
        telemetry.addData("autoTurning","Completed");
    }

    /**
     * This is a Processable method!!! The return is only for Reading stat purpose
     * Should set MotorMode before using this method
     * @param targetAngle targeted Error to Mecanum Turn (sim-stick)
     * @return if the heading is on targetAngle
     */
    private boolean isLocked(double targetAngle){
        boolean onLock = false;
        double curError = getAngluarError(targetAngle);
        double []power = new double[4];

        //whether in range or not
        if(Math.abs(curError)<=RoboMap.Error_Acceptable){
            onLock = true;
            for(int i=0; i<power.length; i++){
                power[i]=0;
            }
        }else{
            double dtemp = p_TurnInput(curError,RoboMap.Kp);
            power[0] = Functions.MecDrive_RightFront(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
            power[1] = Functions.MecDrive_LeftFront(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
            power[2] = Functions.MecDrive_LeftRear(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
            power[3] = Functions.MecDrive_RightRear(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
        }

        //set power
        leftfront.setPower(power[1]);
        leftrear.setPower(power[2]);
        rightfront.setPower(power[0]);
        rightrear.setPower(power[3]);

        //todo telemetry.addData();
        return onLock;
    }

    /**
     * get angular error (not reset so universal)
     * */
    private double getAngluarError(double targetAngle){
        double dtemp = targetAngle - gyro.getIntegratedZValue();
        if(dtemp>=-180&&dtemp<=180){
            return dtemp;
        }else{
            return (dtemp>180)?(dtemp-360):(dtemp+360);
        }
    }

    /**
     * Motor input returned upon Joystick simulation
     * Using only the Kp coefficient
     */
    private double p_TurnInput(double angluarError, double Kp){
        double dtemp = angluarError*(Math.PI/180)*Kp;
        if(dtemp>=-1&&dtemp<=1){
            return dtemp;
        }else{
            return (dtemp>1)?(1):(-1);
        }
    }

    //todo PID full method
}
