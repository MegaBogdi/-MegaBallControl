package org.firstinspires.ftc.teamcode.Manual.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@TeleOp(name = "TestTeleop",group = "Tests")
public class TestTeleop extends CommandOpMode {

    private List<LynxModule> hubs;
    private GamepadEx driver1;
    private CachingServo push1;
    private CachingServo push2;
    private CachingServo hood; // max 0.375

    private CachingCRServo turret;

    private CachingServo park;
    private RevBlinkinLedDriver led;


    @Override
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        driver1 = new GamepadEx(gamepad1);

        push1 = new CachingServo(hardwareMap.get(Servo.class,"pushA"));
        push2 = new CachingServo(hardwareMap.get(Servo.class,"pushB"));
        push2.setDirection(Servo.Direction.REVERSE);
        push1.setPosition(0.02);
        push2.setPosition(0.02);

        hood = new CachingServo(hardwareMap.get(Servo.class,"hood"));
        hood.setDirection(Servo.Direction.REVERSE);
        //hood.setPosition(0);

        park = new CachingServo(hardwareMap.get(Servo.class,"park"));
        park.setDirection(Servo.Direction.REVERSE);
        park.setPosition(0);

        turret = new CachingCRServo(hardwareMap.get(CRServo.class,"turet"));

        led = hardwareMap.get(RevBlinkinLedDriver.class,"led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(()->setPush(0.156));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->setPush(0.056));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()->setPush(push1.getPosition()-0.001));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(()->setPush(push1.getPosition()+0.001));
        driver1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()->push2.setPosition(0));
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(()->push2.setPosition(1));




    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        telemetry.addData("push",push1.getPosition());
        telemetry.update();

    }

    public void setPush(double pos){
        push1.setPosition(pos);
        push2.setPosition(pos);
    }
}
