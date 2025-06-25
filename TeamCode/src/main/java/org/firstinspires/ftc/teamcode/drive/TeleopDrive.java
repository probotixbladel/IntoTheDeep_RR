/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.drive.PIDController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="TeleopDrive", group="Linear OpMode")
//@Disabled
public class TeleopDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public static double gearshift = 0.25;
    public static double turnspeed = 0.4;
    private boolean init = false;;
    public static double pow = 2.3;
    public static double Ks = 0.4;

    private double controller(double x, double gearshift) {
        return (Math.pow(Math.abs(x) * (1-Ks) * gearshift + Ks, pow) * Math.signum(x));
    }
    private GameObjectController Objcon = new GameObjectController();

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        Objcon.init(hardwareMap);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (!init){
                init = true;
            }
            double max;

            double axial   = -controller(gamepad1.left_stick_y, gearshift);
            double lateral =  controller(gamepad1.left_stick_x, gearshift);
            double yaw     =  controller(gamepad1.right_stick_x, turnspeed);


            double leftFrontPower  = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower   = (axial - lateral + yaw);
            double rightBackPower  = (axial + lateral - yaw);

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            if (gamepad1.a == true) {
                gearshift = 0.25;
                turnspeed = 0.4;
            } else if (gamepad1.b == true) {
                gearshift = 0.5;
                turnspeed = 0.65;
            } else if (gamepad1.x == true) {
                gearshift = 0.75;
                turnspeed = 0.8;
            } else if (gamepad1.y == true) {
                gearshift = 1.0;
                turnspeed = 1.0;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            Objcon.update(gamepad2, telemetry);

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}

@Config
class GameObjectController {
    private ElapsedTime time = new ElapsedTime();
    private double last_time = 0;
    private DcMotor erect = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private DcMotorEx heil = null;
    private Servo diffl = null;
    private Servo diffr = null;
    private Servo grabin = null;
    private Servo wrist = null;
    private Servo grabout = null;
    private boolean up = false;
    private boolean pasing = false;
    private boolean lastup = false;
    private boolean grabn = false;
    private boolean lasta = false;
    private boolean lastgrabn = false;
    private boolean completeMid = false;
    private boolean lastPress = false;
    private double lasterect = 0;
    private double espeed = 0;
    private boolean onTheway = false;
    public static double wristDown = 1;
    public static double wristUp = 0.6;
    public static double wait = 250;
    public static double liftpos = 365;
    public static double liftpick = 365;
    public static double posl = 0.22;
    public static double posr = 0.80;
    public static double kp = 0.034;
    public static double ki = 0.00025;
    public static double kd = 0.000004;
    public static double erectspeed = 25.0;
    public static double gotoheil = 0;
    public static double toplift = 2900;
    public static double topheil = 490;
    public static boolean target = false;
    private final double ticksInDegree = 450 / 180.0;
    private double releaseTime = 0;
    public static boolean grab = false;
    private int erection = 55;
    public static int finishErection = 60;
    private double pos = 0;
    private boolean given = false;
    private boolean hasReset = false;
    public static double maxspeed = 5;
    AnalogInput diffleft = null;
    AnalogInput diffright = null;
    private PIDController erectpid = new PIDController(0.01,0.0,0.0005);
    private PIDController heilpid = new PIDController(0.0028, 0.0013, 0.00035);
    public void init(HardwareMap hardwareMap){
        diffleft = hardwareMap.get(AnalogInput.class, "diffleft");
        diffright = hardwareMap.get(AnalogInput.class, "diffright");
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

    public void update(Gamepad gamepad2, Telemetry telemetry){
        double diflpos = diffleft.getVoltage() / 3.3 * 360;
        double difrpos = diffright.getVoltage() / 3.3 * 360;

        //erectpid.setParams(kp, ki, kd);

        if (!lasta & gamepad2.a) {
            if (pasing & !onTheway) {
                pasing = false;
                grab = false;
                given = false;
                grabout.setPosition(0.725);
                onTheway = false;
                up = true;
                diffl.setPosition(0.3);
                diffr.setPosition(0.9);
                erectpid.reset();
                hasReset = false;
                pos = 55;
                erectpid.setParams(0.01,0.0,0.0005);

            }
            else {
                pasing = true;
                grab = false;

            }
        }

        lasta = gamepad2.a;
        if (!pasing) {
            grabout.setPosition(0.725);

            pos += Math.pow(gamepad2.right_trigger, 1.5) * erectspeed - Math.pow(gamepad2.left_trigger, 1.5) * erectspeed;
            if (pos < 55) {
                pos = 55;
            }
            if (pos > 420) {
                pos = 420;
            }
            erection = (int)pos;

            if (gamepad2.left_stick_button && !lastPress) {
                if (up) {
                    up = false;
                } else {
                    up = true;
                }
                lastPress = true;
            } else if (!gamepad2.left_stick_button && lastPress) {
                lastPress = false;
            }

            if (up) {
                if (!completeMid) {
                    if (155 < diflpos & diflpos < 215 & 90 < difrpos & difrpos < 155) {
                        if (172 < diflpos & diflpos < 180 & 111 < difrpos & difrpos < 119) {
                            completeMid = true;
                            diffl.setPosition(0.3);
                            diffr.setPosition(0.9);
                        } else {
                            diffl.setPosition(0.515);
                            diffr.setPosition(0.705);
                        }
                    }
                } else {
                    diffl.setPosition(0.3);
                    diffr.setPosition(0.9);
                }
            } else {
                diffl.setPosition(0.49 - (0.09 * gamepad2.left_stick_x));
                diffr.setPosition(0.7 - (0.09 * gamepad2.left_stick_x));
                completeMid = false;
            }

            if (gamepad2.left_bumper & !lastgrabn) {
                if (grabn) {
                    grabn = false;
                } else {
                    grabn = true;
                }
            }

            if (!up) {
                if (lastup) {
                    grabn = false;
                }
                if (grabn) {
                    grabin.setPosition(1);
                }
                if (!grabn) {
                    grabin.setPosition(0.69);
                }
            } else {
                grabin.setPosition(1);
            }

            if (time.milliseconds() - releaseTime > wait) {
                wrist.setPosition(wristDown);
                liftRight.setTargetPosition((int) liftpos);
                liftLeft.setTargetPosition((int) liftpos);
                gotoheil = 0;

            }

        }else {
            if (grab) {
                grabout.setPosition(0.9);
                grabin.setPosition(0.69);
            }

            if (!grab) {
                grabout.setPosition(0.725);
            }

            if (espeed < maxspeed & erection == finishErection & erect.getCurrentPosition() > -finishErection-5 & erect.getCurrentPosition() < -finishErection+5 & 332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & liftRight.getCurrentPosition() < liftpick+5 & liftRight.getCurrentPosition() > liftpick-5 & heil.getCurrentPosition() > -5 & heil.getCurrentPosition() < 5) {
                grab = true;
                given = true;

                grabout.setPosition(0.9);
                grabin.setPosition(0.69);
            }

            if (!given) {
                if (grab) {
                    grabout.setPosition(0.9);
                    grabin.setPosition(0.69);
                }
                if (!grab) {
                    grabout.setPosition(0.725);
                }
                wrist.setPosition(wristDown);
                liftRight.setTargetPosition((int)liftpick);
                liftLeft.setTargetPosition((int)liftpick);
                diffl.setPosition(0.02);
                diffr.setPosition(1.0);
                erectpid.setParams(0.02,0.008,0.001);
                if (!hasReset) {
                    erectpid.reset();
                    hasReset = true;
                }

                if (332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24) {
                    erection = finishErection;
                } else {
                    erection = 150;
                }

            } else if (gamepad2.b) {
                wrist.setPosition(wristUp);
                grabout.setPosition(0.9);
                liftRight.setTargetPosition((int) toplift);
                liftLeft.setTargetPosition((int) toplift);
                gotoheil = topheil;
                onTheway = true;

            } else if (gamepad2.dpad_left & onTheway & liftRight.getCurrentPosition() > toplift - 10 & liftRight.getCurrentPosition() < toplift + 10) {
                pasing = false;
                grab = false;
                given = false;
                grabout.setPosition(0.725);

                onTheway = false;
                releaseTime = time.milliseconds();
                up = true;
                diffl.setPosition(0.3);
                diffr.setPosition(0.9);
                erectpid.reset();
                hasReset = false;
                pos = 55;
                erectpid.setParams(0.01,0.0,0.0005);

            }
        }

        lastgrabn = gamepad2.left_bumper;
        lastup = up;

        heilpid.setParams(0.0028, 0.0013, 0.00035);
        heil.setPower((Math.sin(Math.toRadians(heil.getCurrentPosition() / ticksInDegree + 45)) * 0.12) + heilpid.update(gotoheil, heil.getCurrentPosition()));

        erect.setPower(erectpid.update(-erection, erect.getCurrentPosition()));
        espeed = lasterect - erect.getCurrentPosition();
        lasterect = erect.getCurrentPosition();


        telemetry.addData("fps: ", 1/(time.seconds()-last_time));
        telemetry.addData("uitschuif goto :", erection);
        telemetry.addData("uitschuif pos  :", erect.getCurrentPosition());
        telemetry.addData("uitschuif speed:", espeed);
        last_time = time.seconds();
    }
}