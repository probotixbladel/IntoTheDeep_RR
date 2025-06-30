package org.firstinspires.ftc.teamcode.autonoom;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Auto Sample", group = "Autonomous")
public class AutoSample extends LinearOpMode {
    public class OBJC {
        private ElapsedTime time = new ElapsedTime();
        private double last_time = 0;
        private double last_times = 0;
        private DcMotor erect = null;
        private DcMotor liftLeft = null;
        private DcMotor liftRight = null;
        private DcMotorEx heil = null;
        private Servo diffl = null;
        private Servo diffr = null;
        private Servo grabin = null;
        private Servo wrist = null;
        private Servo grabout = null;
        private boolean lasta = false;
        private double lasterect = 0;
        private double espeed = 0;
        private boolean onTheway = false;
        public double maxspeed = 20;
        public double wristDown = 0.6;
        public double wristUp = 0.2;
        public double wait = 250;
        public double liftpick = 520;
        public double kp = 0.02;
        public double ki = 0.00;
        public double kd = 0.0;
        public double kf = 0.1;
        public double erectspeed = 25.0;
        public double outClose = 0.75;
        public double outOpen = 0.5;
        public double gotoheil = 0;
        public double toplift = 2900;
        public double topheil = 500;
        private final double ticksInDegree = 450 / 180.0;
        private double releaseTime = 0;
        public  boolean grab = false;
        private int erection = 55;
        public  int finishErection = 125;
        public  int max_fps = 30;
        private double pos = 0;
        private boolean given = false;
        private boolean hasReset = false;
        private boolean grabed = false;
        //public static double maxspeed = 20;
        public   double max_heil = 0.5;
        AnalogInput diffleft = null;
        AnalogInput diffright = null;
        AnalogInput wristAngle = null;
        private PIDController erectpid = new PIDController(0.01,0.0,0.0005);
        private PIDController heilpid = new PIDController(0.008, 0.0, 0.0005);

        public OBJC(HardwareMap hardwareMap) {
            diffleft = hardwareMap.get(AnalogInput.class, "diffleft");
            diffright = hardwareMap.get(AnalogInput.class, "diffright");
            wristAngle = hardwareMap.get(AnalogInput.class, "wristAngle");
            liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
            liftRight = hardwareMap.get(DcMotor.class, "liftRight");
            erect = hardwareMap.get(DcMotor.class, "erect");
            heil = hardwareMap.get(DcMotorEx.class, "heil");
            diffl = hardwareMap.get(Servo.class, "diffLeft");
            diffr = hardwareMap.get(Servo.class, "diffRight");
            grabin = hardwareMap.get(Servo.class, "grab'in");
            wrist = hardwareMap.get(Servo.class, "wrist");
            grabout = hardwareMap.get(Servo.class, "grab'out");
            erect.setPower(0);
            erect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            erect.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            erect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            heil.setPower(0);
            heil.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            heil.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            heil.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRight.setTargetPosition(0);
            liftRight.setPower(1);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftLeft.setPower(1);
            liftLeft.setTargetPosition(0);
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class update implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                heil.setPower(Math.max(-max_heil, Math.min(max_heil,(Math.sin(Math.toRadians(heil.getCurrentPosition() / ticksInDegree + 40)) * kf) + heilpid.update(gotoheil, heil.getCurrentPosition()))));
                erect.setPower(erectpid.update(-erection, erect.getCurrentPosition()));
                espeed = Math.abs(60*(time.seconds()-last_times)*(lasterect - erect.getCurrentPosition()));
                last_times = time.seconds();
                lasterect = erect.getCurrentPosition();
                packet.put("espeed", espeed);
                return true;
            }
        }
        public Action update() {
            return new update();
        }

        public class Grab implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    erection = 300;
                    diffl.setPosition(0.49 - (0.09 * gamepad2.left_stick_x));
                    diffr.setPosition(0.7 - (0.09 * gamepad2.left_stick_x));
                    liftRight.setTargetPosition((int) liftpick);
                    liftLeft.setTargetPosition((int) liftpick);
                    grabin.setPosition(0.69);
                    initialized = true;

                }
                double epos = erect.getCurrentPosition();
                packet.put("liftPos", epos);
                if (epos > 295 & epos < 305 & espeed < maxspeed & !grabed) {
                    //return true;
                    grabin.setPosition(1);
                    grabed = true;
                }
                double diflpos = diffleft.getVoltage() / 3.3 * 360;
                double difrpos = diffright.getVoltage() / 3.3 * 360;
                if (grabed) {
                    if (espeed < maxspeed & erection == finishErection & erect.getCurrentPosition() > -finishErection-7 & erect.getCurrentPosition() < -finishErection+7 & 332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & liftRight.getCurrentPosition() < liftpick+5 & liftRight.getCurrentPosition() > liftpick-5 & heil.getCurrentPosition() > -8 & heil.getCurrentPosition() < 8) {
                        grab = true;
                        given = true;

                        grabout.setPosition(outClose);
                        grabin.setPosition(0.69);
                        return false;
                    }
                }
                if (!given) {
                    grabout.setPosition(outClose);
                    if (grab) {
                        grabout.setPosition(outClose);
                        grabin.setPosition(0.69);
                    }
                    if (!grab) {
                        grabout.setPosition(outOpen);
                    }
                    wrist.setPosition(wristDown);
                    liftRight.setTargetPosition((int)liftpick);
                    liftLeft.setTargetPosition((int)liftpick);
                    diffl.setPosition(0.02);
                    diffr.setPosition(1.0);

                    if (!hasReset) {
                        erectpid.setParams(0.02,0.008,0.001);
                        erectpid.reset();
                        hasReset = true;
                    }

                    if (332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & espeed < maxspeed & erection != finishErection) {
                        erection = finishErection;
                        erectpid.reset();
                        //erectpid.setParams(0.01,0.00025,0.00008);
                        erectpid.setParams(kp,ki,kd);
                    } else if (erection != finishErection) {
                        erection = 215;
                    }
                }
                return true;
            }
        }
        public Action Grab(){
            return new Grab();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        OBJC Objcon = new OBJC(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        Objcon.Grab(),
                        Objcon.update()
                )
        );
    }
}