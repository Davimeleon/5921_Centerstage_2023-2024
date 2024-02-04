package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.BasicPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "autoRedClose")
public class autoRedClose extends LinearOpMode {

    protected DcMotor lf;
    protected DcMotor rf;
    protected DcMotor lb;
    protected DcMotor rb;
    protected DcMotor arm;
    protected DcMotor intake;
    protected Servo armServo;
    protected Servo trapdoor;
    OpenCvCamera camera;
    PipelineRed pipeline = new PipelineRed();

    //BasicPipeline pipeline = new BasicPipeline();

    enum propPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }


    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotor.class, "left_front");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        lb = hardwareMap.get(DcMotor.class, "left_back");
        rb = hardwareMap.get(DcMotor.class, "right_back");

        arm = hardwareMap.get(DcMotor.class, "arm");

        intake = hardwareMap.get(DcMotor.class, "intake");
        armServo = hardwareMap.get(Servo.class, "armServo");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armServo.setPosition(0.1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());

            if (pipeline.getPropAreaAttr() > 20000 && pipeline.getJunctionPoint().x > 545){
                telemetry.addLine("Box Placement: Right");
            }
            else if (pipeline.getPropAreaAttr() > 20000) { //Middle Path
                telemetry.addLine("Box Placement: Middle");
            }
            else{
                telemetry.addLine("Box Placement: Left");
            }
            telemetry.update();
        }

        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();

        //Full Battery Power Constant = 0.80
        //13.7 Battery Constant = 0.87
        //13.6 Battery Constant = 0.89
        //13.39 Battery Constant = 0.9
        //13.16 BatteryConstant = 0.91
        //12.96 Battery Constant = 0.92
        //12.85 Battery Constant = 0.93
        //12.78 = 0.94
        double batteryConstant = 0.914;

        if (propArea > 20000 && propX > 545){ //Right Path
            moveForward(-0.3, (int)(300 * batteryConstant));
            moveStrafing(0.3, (int)(825 * batteryConstant)); //Strafe Left
            sleep(100);
            moveForward(-0.3, (int)(1450 * batteryConstant));
            sleep(250);
            moveForward(0.2, (int)(500 * batteryConstant));
            intake.setPower(-0.6);
            sleep(1750);
            intake.setPower(0);
            sleep(250);
            moveForward(0.3, (int)(325 * batteryConstant));
            sleep(100);
            moveTurning(-0.2, (int)(2550 * batteryConstant)); //Turn Left
            armServo.setPosition(0.05);
            arm.setPower(0.5);
            sleep((int)(2600 * batteryConstant));
            arm.setPower(0);
            moveStrafing(-0.3, (int)(175 * batteryConstant)); //Strafe Right
            moveForward(-0.3, (int)(2000 * batteryConstant));
        }

        else if (propArea > 20000){ //Middle Path
            moveForward(-0.3, (int)(2000 * batteryConstant));
            sleep(200);
            moveForward(0.3, (int)(200 * batteryConstant));
            intake.setPower(-0.6);
            sleep(1500);
            intake.setPower(0);
            sleep(200);
            moveForward(0.4, (int)(300 * batteryConstant));
            moveTurning(-0.2, (int)(2625 * batteryConstant)); //Turn Left
            armServo.setPosition(0.05);
            arm.setPower(0.5);
            sleep((int)(2600 * batteryConstant));
            arm.setPower(0);
            moveForward(-0.3, (int)(1500 * batteryConstant));
            sleep(500);
            moveStrafing(-0.3, (int)(400 * batteryConstant)); //Strafe Right
            sleep(500);
            moveForward(-0.2, (int)(1430 * batteryConstant));

            //Drop

        }

        else{ //Left Path
            moveForward(-0.3, (int)(1850 * batteryConstant));
            moveTurning(0.2, (int)(2750 * batteryConstant));
            sleep(200);
            moveForward(-0.3, (int)(400 * batteryConstant));
            sleep(100);
            moveForward(0.3, (int)(220 * batteryConstant));
            intake.setPower(-0.6);
            sleep((int)(1500 * batteryConstant));
            intake.setPower(0);
            sleep(200);
            moveForward(0.3, (int)(500 * batteryConstant));
            sleep(300);
            moveTurning(-0.2, (int)(5615 * batteryConstant));
            sleep(300);
            arm.setPower(0.5);
            sleep((int)(2600 * batteryConstant));
            arm.setPower(0);
            moveForward(-0.3, (int)(1550 * batteryConstant));
            sleep(500);
            moveStrafing(-0.2, (int)(440 * batteryConstant)); //Strafe Right
            sleep(500);
            moveForward(-0.20, (int)(1130 * batteryConstant));
        }

        armServo.setPosition(0.05);

        //DROP
        trapdoor.setPosition(0.6);
        sleep(3000);
        moveForward(0.2, (int)(700 * batteryConstant));
        moveStrafing(0.3, (int)(1750 * batteryConstant));
        sleep(1000);
        trapdoor.setPosition(0.35);
        arm.setPower(-0.5);
        sleep((int)(2250 * batteryConstant));
        arm.setPower(0);


        //right_front.setPower(1);
        //left_front.setPower(1);
        //right_back.setPower(1);
        //left_back.setPower(1);
        //sleep(1000);
        //right_front.setPower(0);
        //left_front.setPower(0);
        //right_back.setPower(0);
        // left_back.setPower(0);

        /*arm.setPower(0.075);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0.075);
        //double propX = pipeline.getJunctionPoint().x;
        //double propArea = pipeline.getPropAreaAttr();

        moveForward(0.25, 2400);

        telemetry.addLine("Left spike mark");
        telemetry.update();
        moveTurning(-0.25, 2200);
        moveForward(0.5, 100);
        // eject pixel
        intake.setPower(-1);
        sleep(1000);
        intake.setPower(0);
        // go to backboard and score, evading pixel
        moveTurning(0.25, 4400);
        moveStrafing(0.25, 2000);
        moveForward(-0.25, 3000);
        moveStrafing(-0.25, 2000);*/

        /*if (propArea < 10000) { // none detected we assume left spike mark
            telemetry.addLine("Left spike mark");
            telemetry.update();
            moveTurning(-0.25, 2200);
            moveForward(0.5, 100);
            // eject pixel
            intake.setPower(-1);
            sleep(1000);
            intake.setPower(0);
            // go to backboard and score, evading pixel
            moveTurning(0.25, 4400);
            moveStrafing(0.25, 2000);
            moveForward(-0.25, 3000);
            moveStrafing(-0.25, 2000);

        } else if (propX > 600) { // right spike mark
            telemetry.addLine("Right spike mark");
            telemetry.update();
            // line up with mark
            moveTurning(0.25, 2200);
            moveForward(0.25, 300);
            // eject pixel
            intake.setPower(-0.25);
            sleep(1000);
            intake.setPower(0);
            //in case it falls vertically
            moveForward(0.25, 100);
            // go to backboard and score
            moveForward(-0.25, 4000);
        } else { // middle spike mark
            telemetry.addLine("Middle spike mark");
            telemetry.update();
            moveForward(0.25, 400);
            moveForward(-0.25, 400);
            // eject pixel- we're already there
            intake.setPower(-0.25);
            sleep(1000);
            intake.setPower(0);
            moveForward(0.25, 100);
            // go to backboard
            moveForward(-0.25, 400);
            moveTurning(0.25, 2500);
            moveForward(-0.25, 3200);
        }

        servo.setPosition(0);
        sleep(400);
        arm.setTargetPosition(700);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0/4);
        sleep(1000);
        servo.setPosition(0.7);*/

        sleep(100000);
    }

    public void moveTurning(double power, int time) {
        rf.setPower(power);
        lf.setPower(power);
        rb.setPower(power);
        lb.setPower(power);
        sleep(time);
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);

    }

    public void moveStrafing(double power, int time){
        rf.setPower(-power);
        lf.setPower(-power);
        rb.setPower(power);
        lb.setPower(power);
        sleep(time);
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);

    }

    public void moveForward(double power, int time) {
        rf.setPower(-power);
        lf.setPower(power);
        rb.setPower(-power);
        lb.setPower(power);
        sleep(time);
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
}