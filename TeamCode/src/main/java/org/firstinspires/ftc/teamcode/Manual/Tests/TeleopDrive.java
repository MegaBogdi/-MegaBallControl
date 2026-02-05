package org.firstinspires.ftc.teamcode.Manual.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@TeleOp(name = "TeleopDrive",group = "Tests")
public class TeleopDrive extends CommandOpMode {

    private List<LynxModule> hubs;
    private GamepadEx driver1;
    private Follower follower;
    private DriveSubsystem chassis;

    @Override
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        driver1 = new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);
        chassis = new DriveSubsystem(hardwareMap);




    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        chassis.updateSpeeds(driver1.getLeftY(),driver1.getLeftX(),driver1.getRightX(),follower.getHeading());
        telemetry.addData("heading:", follower.getHeading());
        telemetry.update();

    }

}
