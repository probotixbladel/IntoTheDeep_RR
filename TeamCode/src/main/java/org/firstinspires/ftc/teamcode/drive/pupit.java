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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name="pupit", group="Linear OpMode")
//@Disabled
public class pupit extends LinearOpMode {
    private DcMotor erect = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private DcMotorEx heil = null;
    private Servo diffl = null;
    private Servo diffr = null;
    private Servo grabin = null;
    private Servo wrist = null;
    private Servo grabout = null;
    AnalogInput diffleft = null;
    AnalogInput diffright = null;
    AnalogInput wristAngle = null;
    public static double wrist_pos = 0.5;
    public static double grabin_pos = 0.69;
    public static double grabout_pos = 0.5;
    public static double diffl_pos = 0.49;
    public static double diffr_pos = 0.7;

    @Override
    public void runOpMode() {
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
        liftRight.setPower(0);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setPower(0);
        liftLeft.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            double diflpos = diffleft.getVoltage() / 3.3 * 360;
            double difrpos = diffright.getVoltage() / 3.3 * 360;
            double wrista = wristAngle.getVoltage() / 3.3 * 360;
            grabin.setPosition(grabin_pos);
            grabout.setPosition(grabout_pos);
            wrist.setPosition(wrist_pos);
            diffr.setPosition(diffr_pos);
            diffl.setPosition(diffl_pos);

            telemetry.addData("erect pos ", erect.getCurrentPosition());
            telemetry.addData("lift left ", liftLeft.getCurrentPosition());
            telemetry.addData("lift right", liftRight.getCurrentPosition());
            telemetry.addData("heil pos  ", heil.getCurrentPosition());

            telemetry.addData("diflpos :", diflpos);
            telemetry.addData("difrpos :", difrpos);
            telemetry.addData("wrista :", wrista);

            telemetry.update();
        }
    }
}