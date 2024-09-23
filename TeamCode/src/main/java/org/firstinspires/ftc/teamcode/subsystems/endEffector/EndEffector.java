package org.firstinspires.ftc.teamcode.subsystems.endEffector;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.SampleColors;
import org.firstinspires.ftc.teamcode.util.StepperServo;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class EndEffector {
    ContinuousServo servo1;
    ContinuousServo servo2;
    StepperServo elbow;
    RevColorSensorV3 colorSensor;

    ElapsedTime sensorTimeout;

    public EndEffector(ContinuousServo s1, ContinuousServo s2, StepperServo s3, RevColorSensorV3 sensor) {
        servo1 = s1;
        servo2 = s2;
        elbow = s3;
        colorSensor = sensor;

        sensorTimeout = new ElapsedTime();

        colorSensor.initialize();
        colorSensor.enableLed(true);
    }

    public void setPower(float p) {
        servo1.servo.setPower(p);
        servo2.servo.setPower(p);
    }

    public void setAngle(float angle) {
        elbow.setAngle(angle);
    }

    public void startIntake() {
        setPower(1);
    }

    public void stopIntake() {
        setPower(0);
    }

    public void smartStopIntake(SampleColors... colors) {
        SampleColors s = detectSample();
        if (Arrays.stream(colors).anyMatch(x -> x == s )) {
            stopIntake();
        } else if (s != null) {
            ejectSample();
        }
    }

    public void ejectSample() {
        setPower((float) -0.5);
    }

    public SampleColors detectSample() {
        if (sensorTimeout.time(TimeUnit.SECONDS) < 1) {
            return null;
        }

        if (colorSensor.blue() > 0.5) {
            return SampleColors.BLUE;
        } else if (colorSensor.red() > 0.5) {
            return SampleColors.RED;
        } else if (colorSensor.getDistance(DistanceUnit.MM) > 3) {
            return SampleColors.YELLOW;
        }

        sensorTimeout.reset();

        return null;
    }
}