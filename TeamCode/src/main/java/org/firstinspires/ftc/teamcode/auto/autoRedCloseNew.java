package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.auto.BasicPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "autoRedCloseNew")
public class autoRedCloseNew extends LinearOpMode {

    protected DcMotorEx lf;
    protected DcMotorEx rf;
    protected DcMotorEx lb;
    protected DcMotorEx rb;
    protected DcMotor arm;
    protected DcMotor intake;
    protected Servo armServo;
    protected Servo trapdoor;
    OpenCvCamera camera;
    PipelineBlue pipeline = new PipelineBlue();

    ArrayList<DcMotorEx> driveMotors = new ArrayList<>();

    //BasicPipeline pipeline = new BasicPipeline();

    enum propPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }


    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotorEx.class, "left_front");
        rf = hardwareMap.get(DcMotorEx.class, "right_front");
        lb = hardwareMap.get(DcMotorEx.class, "left_back");
        rb = hardwareMap.get(DcMotorEx.class, "right_back");

        lf.setDirection(Constants.motorDirections.get("left_front"));
        lb.setDirection(Constants.motorDirections.get("left_back"));
        rf.setDirection(Constants.motorDirections.get("right_front"));
        rb.setDirection(Constants.motorDirections.get("right_back"));

        driveMotors.add(lf);
        driveMotors.add(lb);
        driveMotors.add(rf);
        driveMotors.add(rb);

        arm = hardwareMap.get(DcMotor.class, "arm");

        intake = hardwareMap.get(DcMotor.class, "intake");
        armServo = hardwareMap.get(Servo.class, "armServo");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        armServo.setPosition(0.45);

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


        sleep(1500);
        armServo.setPosition(0.45);

        sleep(1500);


        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());

            if (pipeline.getPropAreaAttr() > 30000 && pipeline.getJunctionPoint().x > 750) {
                telemetry.addLine("Box Placement: Right");
            } else if (pipeline.getPropAreaAttr() > 25000) { //Middle Path
                telemetry.addLine("Box Placement: Middle");
            } else {
                telemetry.addLine("Box Placement: Left");
            }
            telemetry.update();
        }

        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();
        double autoPower = 500;

        if (propArea > 30000 && propX > 750) { //Right Path
            goTo(1280, 0, 0, autoPower, true);
            turn(90);
            goTo(350, 0, 0, autoPower, true);
            goTo(-90, 0, 0, autoPower, true);
            intake.setPower(-0.7);
            sleep(2000);
            intake.setPower(0);
            sleep(250);
            goTo(40, 0, 0, autoPower, true);
            goTo(-400, 0, 0, autoPower, true);
            turn(180);


        } else if (propArea > 25000) { //Middle Path
            goTo(1420, 0, 0, autoPower, true);
            goTo(-160, 0, 0, autoPower, true);
            intake.setPower(-0.7);
            sleep(1750);
            intake.setPower(0);
            sleep(250);
            goTo(50, 0, 0, autoPower, true);
            goTo(-250, 0, 0, autoPower, true);
            turn(-90);
            goTo(1000, 0, 0, autoPower, true);

        } else { //Left Path
            goTo(550, 0, 0, autoPower, true);
            goTo(0, 180, 0, autoPower, true);
            goTo(820, 0, 0, autoPower, true);
            turn(-90);
            goTo(125, 0, 0, autoPower, true);
            goTo(-60, 0, 0, autoPower, true);
            intake.setPower(-0.7);
            sleep(1750);
            intake.setPower(0);
            sleep(250);
            goTo(40, 0, 0, autoPower, true);
            goTo(-130, 0, 0, autoPower, true);
            goTo(0, 725, 0, autoPower, true);
            goTo(1500, 0, 0, autoPower, true);
            goTo(0, -1050, 0, autoPower, true);
        }
    }

    public void goTo(double forward, double strafe, double yaw, double powercoef, boolean waitToFinish) {
        /**
         * Moves robot-centrically.  All is in ticks except yaw, which is approximately degrees such that 90 turns the robot 90 degrees clockwise.
         */
        yaw *= 11;
        double leftFrontPower = powercoef * (forward + strafe + yaw);
        double rightFrontPower = powercoef * (forward - strafe - yaw);
        double leftBackPower = powercoef * (forward - strafe + yaw);
        double rightBackPower = powercoef * (forward + strafe - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 2000) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
            leftFrontPower *= powercoef;
            leftBackPower *= powercoef;
            rightFrontPower *= powercoef;
            rightBackPower *= powercoef;
        }

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        lf.setTargetPosition((int) (forward + strafe + yaw));
        lb.setTargetPosition((int) (forward - strafe + yaw));
        rf.setTargetPosition((int) (forward - strafe - yaw));
        rb.setTargetPosition((int) (forward + strafe - yaw));

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        lf.setVelocity(leftFrontPower);
        lb.setVelocity(leftBackPower);
        rf.setVelocity(rightFrontPower);
        rb.setVelocity(rightBackPower);

        if (waitToFinish) while (lf.isBusy()) {
            telemetry.addData("lf power: ", leftFrontPower);
            telemetry.addData("lf tgt position: ", lf.getTargetPosition());
            telemetry.addData("lf position: ", lf.getCurrentPosition());
            telemetry.update();
        }
    }

    public void goForward(double inches, double power){
        goTo(inches * 20, 0, 0, power, true);
    }

    public void turn (double degrees){
        goTo(0, 0, degrees / 90 * 93, 500, true);
    }
}