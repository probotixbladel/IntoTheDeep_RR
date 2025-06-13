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
@TeleOp(name="Teleop_test", group="Linear OpMode")
//@Disabled
public class Teleop_test extends LinearOpMode {

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
    //init vars:
    private DcMotor slideoutMotor, liftLeft, liftRight;
    private Servo diffl, diffr, grabin, wrist, grabout;
    private boolean up, pasing, lastup, grabn, lasta, lastgrabn, completeMid, lastPress, onTheway, given;
    private double lasterect, espeed, pos;

    private DcMotorEx outputMotor = null;

    AnalogInput diffleft = null;
    AnalogInput diffright = null;
    private PIDController slideoutPID = new PIDController(Constants.SLIDEOUT_P, Constants.SLIDEOUT_I, Constants.SLIDEOUT_D);
    private PIDController outputPID = new PIDController(Constants.OUTPUT_P, Constants.OUTPUT_I, Constants.OUTPUT_D);

    // Use these as local variables instead of class fields:
    // double gotoheil = Constants.OUTPUT_ARM_START_POSITION;
    // double topheil = Constants.OUTPUT_ARM_TOP_POSITION;
    // boolean target = Constants.TARGET;
    // final double ticksInDegree = Constants.TICKS_IN_DEGREE;
    // boolean grab = Constants.GRAB;
    // int erection = Constants.SLIDEOUT_ARM_START_POSITION;
    // int finishErection = Constants.SLIDEOUT_ARM_END_POSITION;
    // double grabpos = Constants.GRAB_POSITION;

    public void init(HardwareMap hardwareMap) {
        //TODO
        //change hardwareMap on the android thingie from the commented name to the current name
        diffleft = hardwareMap.get(AnalogInput.class, "diffleft"); //kijk even naar deze, nu zijn er diffLeft en diffleft maar wat is wat.
        diffright = hardwareMap.get(AnalogInput.class, "diffright"); //kijk even naar deze, nu zijn er diffRight en diffright maar wat is wat.
        diffl = hardwareMap.get(Servo.class, "diffLeft");  //kijk even naar deze, nu zijn er diffLeft en diffleft maar wat is wat.
        diffr = hardwareMap.get(Servo.class, "diffRight"); //kijk even naar deze, nu zijn er diffRight en diffright maar wat is wat.


        liftLeftMotor = hardwareMap.get(DcMotor.class, "liftLeftMotor"); //previously "liftLeft"
        liftRightMotor = hardwareMap.get(DcMotor.class, "liftRightMotor"); //previously "liftRight"
        slideoutMotor = hardwareMap.get(DcMotor.class, "slideoutMotor"); //previously "erect"
        outputMotor = hardwareMap.get(DcMotorEx.class, "outputMotor");  //previously "heil"

        grabInServo = hardwareMap.get(Servo.class, "grabInServo"); //previously "grab`in"
        wristServo = hardwareMap.get(Servo.class, "wristServo"); //previously "wrist"
        grabOutServo = hardwareMap.get(Servo.class, "grabOutServo"); //previously "grab`out"


        //initialize the motor vars.
        slideoutMotor.setPower(0);
        slideoutMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideoutMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideoutMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        outputMotor.setPower(0);
        outputMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outputMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outputMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        liftRightMotor.setTargetPosition(0);
        liftRightMotor.setPower(1);
        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeftMotor.setPower(1);
        liftLeftMotor.setTargetPosition(0);
        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    
    public void update(Gamepad gamepad2, Telemetry telemetry){
        double diflpos = diffleft.getVoltage() / 3.3 * 360;
        double difrpos = diffright.getVoltage() / 3.3 * 360;
        slideoutPID.setParams(Constants.KP, Constants.KI, Constants.KD);

        if (!lasta & gamepad2.a) {
            if (pasing) {pasing = false;}
            else {
                pasing = true;
                grab = false;
            }
        }

        lasta = gamepad2.a;
        if (!pasing) {
            Constants.OUTPUT_ARM_START_POSITION = 0;
            grabout.setPosition(0.725);

            pos += Math.pow(gamepad2.right_trigger, 1.5) * Constants.SLIDEOUT_ARM_SPEED - Math.pow(gamepad2.left_trigger, 1.5) * Constants.SLIDEOUT_ARM_SPEED;
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
                            diffl.setPosition(0.2);
                            diffr.setPosition(0.8);
                        } else {
                            diffl.setPosition(0.515);
                            diffr.setPosition(0.705);
                        }
                    }
                } else {
                    diffl.setPosition(0.2);
                    diffr.setPosition(0.8);
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

            liftRightMotor.setTargetPosition((int) Constants.LIFT_POSITION);
            liftLeftMotor.setTargetPosition((int) Constants.LIFT_POSITION);

        }else {
            if (grab) {
                grabout.setPosition(0.9);
                grabin.setPosition(0.69);
            }

            if (!grab) {
                grabout.setPosition(0.725);
            }

            if (erect.getCurrentPosition() > -finishErection-5 & erect.getCurrentPosition() < -finishErection+5 & 332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & liftRight.getCurrentPosition() < 385 & liftRight.getCurrentPosition() > 375 & heil.getCurrentPosition() > -5 & heil.getCurrentPosition() < 5) {
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
                wrist.setPosition(1);
                liftRight.setTargetPosition(380);
                liftLeft.setTargetPosition(380);
                diffl.setPosition(0.02);
                diffr.setPosition(1.0);

                if (332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24) {
                    erection = finishErection;
                } else {
                    erection = 100;
                }

            } else if (gamepad2.y) {
                wrist.setPosition(grabpos);
                grabout.setPosition(0.9);
                liftRight.setTargetPosition((int) Constants.TOP_LIFT);
                liftLeft.setTargetPosition((int) Constants.TOP_LIFT);
                Constants.OUTPUT_ARM_START_POSITION = Constants.TOP_LIFT;
                onTheway = true;

            } else if (gamepad2.dpad_left & onTheway & liftRight.getCurrentPosition() > Constants.TOP_LIFT - 10 & liftRight.getCurrentPosition() < Constants.TOP_LIFT + 10) {
                pasing = false;
                grab = false;
                given = false;
                grabout.setPosition(0.725);
                wrist.setPosition(0.1);
                grabout.setPosition(0.725);
                onTheway = false;

            }
        }

        lastgrabn = gamepad2.left_bumper;
        lastup = up;

        outputPID.setParams(Constants.OUTPUT_P, Constants.OUTPUT_I, Constants.OUTPUT_D);
        outputMotor.setPower((Math.sin(Math.toRadians(heil.getCurrentPosition() / ticksInDegree + 45)) * 0.12) + outputPID.update(Constants.SLIDEOUT_ARM_START_POSITION, outputMotor.getCurrentPosition()));

        slideoutMotor.setPower(slideoutPID.update(-erection, slideoutMotor.getCurrentPosition()));
        espeed = lasterect - slideoutMotor.getCurrentPosition();
        lasterect = slideoutMotor.getCurrentPosition();

        telemetry.addData("uitschuif goto :", Constants.SLIDEOUT_ARM_START_POSITION);
        telemetry.addData("uitschuif pos  :", slideoutMotor.getCurrentPosition());
        telemetry.addData("uitschuif speed:", espeed);
    }
}