package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitRGB;

@Autonomous(name="AutonomousMode", group="Linear Opmode")
@Disabled
public class AutonomousMode extends LinearOpMode {

    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor testMotor;

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        leftfront  = hardwareMap.get(DcMotor.class, "motor2");
        rightfront = hardwareMap.get(DcMotor.class, "motor1");
        leftrear = hardwareMap.get(DcMotor.class, "motor3");
        rightrear = hardwareMap.get(DcMotor.class,"motor4");
        testMotor = hardwareMap.get(DcMotor.class,"motor5");

        //leftfront.setDirection(DcMotor.Direction.FORWARD);
        //leftrear.setDirection(DcMotor.Direction.FORWARD);
        //rightfront.setDirection(DcMotor.Direction.REVERSE);
        //rightrear.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        timer.reset();

        leftfront.setPower(0.25);
        leftrear.setPower(0.25);
        rightfront.setPower(0.25);
        rightrear.setPower(0.25);

        sleep(1000);

        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);

        sleep(3000);

        //testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //testMotor.setTargetPosition(testMotor.getCurrentPosition()+500);
        //testMotor.setPower(0.33);
        while (opModeIsActive()) {
            int position = testMotor.getCurrentPosition();
            telemetry.addData("Encoder",position);

            idle();
        }

    }
}
