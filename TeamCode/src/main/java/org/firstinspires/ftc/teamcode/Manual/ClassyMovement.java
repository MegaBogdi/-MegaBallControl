package org.firstinspires.ftc.teamcode.Manual;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Gains;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.BooleanSupplier;

@Config
@TeleOp(name = "ClassyMovement")
public class ClassyMovement extends CommandOpMode {
    private List<LynxModule> hubs;
    private boolean ALLIANCE;
    private double kTurn;
    private ElapsedTime timer;
    private IOSubsystem IO;

    private DriveSubsystem chassis;

    GamepadEx driver2;

    GamepadEx driver1;

    public static double unghi = 0.0;

    public double tPrev;
    public double vPrev;


    IMU imu;

    private Telemetry telemetryA;

    private double[] interpValues;

    private Pose curPose;

    double rx;

    private boolean go_on = false;
    private boolean controlOverride = false;

    private double error;
    private Follower follower;

    private BooleanSupplier found;
    private boolean succesDemand;
    private double headingOffset = 0;
    double alpha;



    /**
     * Code to run during the initialization phase of the OpMode.
     */
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        imu.resetYaw();

        chassis = new DriveSubsystem(hardwareMap);
        IO = new IOSubsystem(hardwareMap);

        follower = Constants.createFollower(hardwareMap); // ALIANCE = TEAM IS READ;
        if (ALLIANCE){follower.setStartingPose(new Pose(97.19626168224299,9.24299065420562,Math.toRadians(90)));}  //new Pose(97.19626168224299,34.24299065420562,Math.toRadians(270))
        else{follower.setStartingPose(new Pose(97.19626168224299,9.24299065420562,Math.toRadians(90)));}//new Pose(114.54205607476636,9.981308411214965,Math.toRadians(270))
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        register(IO);
        IO.close();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        Command quickPush = new SequentialCommandGroup(
                new InstantCommand(() -> IO.setPush(IO.PUSH_MAX_LIMIT)),
                new WaitCommand(35),
                new InstantCommand(() -> IO.setPush(IO.PUSH_MIN_LIMIT)),
                new WaitCommand(35),
                new InstantCommand(() -> IO.regist_release())
        );


        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                //.whenActive(()->IO.setTargetTurretRads(IO.tick2rads(Gains.TurretGains.MAX_TICKS)));
                //.whenActive(()->IO.setMotorRPM(2300));
                        .whenActive(()->IO.setMotorRPM(IO.returnTargetRPM()+500));
                                //.whenActive(()->IO.setHood(IO.SERVO_MAX_LIMIT));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                        //.whenActive(()->IO.setTargetTurretRads(IO.tick2rads(Gains.TurretGains.MIN_TICKS)));
                .whenActive(()->IO.setMotorRPM(IO.returnTargetRPM()-500));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
//                .whenActive(()->IO.setMotorRPM(2250));           // 3500
                //.whenActive(() -> IO.setMotorRPM(2500));
                .whenActive(new SequentialCommandGroup(new InstantCommand(()->IO.setPush(IO.PUSH_MAX_LIMIT)),new WaitCommand(35),new InstantCommand(()->IO.setPush(IO.PUSH_MIN_LIMIT)),new WaitCommand(35),new InstantCommand(()->IO.regist_release())));
                        //.whenActive(new ScheduleCommand(quickPush));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)    // 3400     86 28
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                //.whenActive(()->IO.setTargetTurretRads(-IO.returnTargetTuret("rads")));
                .whenActive(()->IO.setHood(IO.SERVO_MAX_LIMIT));
        Command correctify =
                new SequentialCommandGroup(
                        new InstantCommand(() -> IO.open()),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> IO.isSorterReady()),
                                new RepeatCommand(new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.climb1inDir()),
                                        new WaitCommand(500)
                                ))
                        ),
                        new ConditionalCommand(
                                new InstantCommand(),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.climb1inDir()),
                                        new WaitCommand(500)
                                ),
                                () -> IO.isSorterReady()
                        ), // Catch last rezidual climb
                        new InstantCommand(() -> IO.ALL[0] = 0)
                );

        driver1.getGamepadButton(GamepadKeys.Button.BACK)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.A)))
                .whenActive(correctify);
        driver1.getGamepadButton(GamepadKeys.Button.BACK)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.B)))
                .whenActive(() -> IO.rectify());


        Command changeHeading = new InstantCommand(() -> {
            Pose cur = follower.getPose();
            headingOffset = follower.getPose().getHeading() - Math.toRadians(90);
            telemetryA.addLine("reading");
            telemetryA.update();
        });

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.5))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        driver1.getGamepadButton(GamepadKeys.Button.START)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.Y)))
                .whenActive(changeHeading)
                .whenActive(()->gamepad1.rumble(400));

        driver1.getGamepadButton(GamepadKeys.Button.START)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.X)))
                .whenActive(() -> follower.setPose(new Pose(102.2056074766355, 9.196261682242984, Math.toRadians(270)))) //new Pose(89.64485981308411,9.196261682242984,Math.toRadians(270))
                .whenActive(()->gamepad1.rumble(400));
        driver1.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.START)))
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(new InstantCommand(() -> IO.climb()));

        Command startIntake =
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> IO.start_intake()),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new WaitCommand(50),
                                                new InstantCommand(() -> IO.regist()),
                                                new InstantCommand(() -> IO.climb()),
                                                new WaitUntilCommand(() -> !IO.isOcupied())
                                        ),
                                        new InstantCommand(() -> {
                                        }),
                                        () -> IO.isOcupied() // returns boolean.
                                )
                        ),
                        new InstantCommand(()->{gamepad1.rumble(20);}),
                        () -> IO.ocupied() < 3
                );
        Command stopIntake = new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });


        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ConditionalCommand(new InstantCommand(() -> IO.open()), new InstantCommand(), () -> IO.ocupied() < 3))
                .whileHeld(startIntake)
                .whenReleased(stopIntake);


        Command interp = new InstantCommand(() -> {
            double dist = IO.getDistanceOdom(follower.getPose());
            interpValues = IO.getInterpolatedValues(dist);
            IO.setHood(interpValues[1]);
            IO.setMotorRPM(interpValues[0]);
        });
        Command Demand = new SequentialCommandGroup(
                new InstantCommand(() -> succesDemand = IO.getDemanded()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> IO.isSorterReady() && IO.isRPMready()),
                                quickPush
                        ),
                        new InstantCommand(() -> IO.demand_index += 1),
                        () -> succesDemand
                )
        );
        Command autoAim = new InstantCommand(() -> {
            alpha = IO.getAngle(follower.getPose());
            double heading = AngleUnit.normalizeRadians(follower.getPose().getHeading()+Math.toRadians(180));
            IO.setTargetTurretRads(-(AngleUnit.normalizeRadians(heading-alpha)));
        });
        Command autoOutake = new ParallelCommandGroup(
                autoAim,interp,
                new ConditionalCommand(
                        Demand,
                        new InstantCommand(()->{}),
                        () -> IO.isTurretReady(alpha,follower)
                )

        );
        Command stopAutoOutake = new InstantCommand(() -> {
            IO.setMotorRPM(0);
            IO.setHood(0.1);
            //IO.setTargetTurretRads(0);
            controlOverride = false;
        });


        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.1)
                .whenActive(new InstantCommand(() -> IO.close()))
                .whileActiveContinuous(autoOutake)
                .whenInactive(stopAutoOutake);

        while (opModeInInit()) {
            telemetry.addLine("CHOOSE ALLIANCE");

            if (gamepad1.dpad_up) {
                ALLIANCE = true;
            }
            if (gamepad1.dpad_down) {
                ALLIANCE = false;
            }

            telemetryA.addLine((ALLIANCE ? ">" : "") + "RED ALLIANCE");
            telemetryA.addLine((ALLIANCE ? "" : ">") + "BLUE ALLIANCE");
            telemetryA.update();

        }
        schedule(new InstantCommand(() -> IO.setALIANCE(ALLIANCE)));

    }


    /**
     * Loop that runs for every iteration of the OpMode after start is pressed.
     */
    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();
        follower.update();
        IO.update_sep_pid();
        IO.update_RPM_pid();
        IO.update_turret_pid();
        //IO.testThePower(1);
        //IO.testTS();
        //float[] RGB =IO.getRGB();
        //follower.setTeleOpDrive(driver1.getLeftY(),driver1.getLeftX(),driver1.getRightX(),false,headingOffset);
        telemetryA.addData("driver1:","leftY=%.3f leftX%.3f rightX%.3f",driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX());
        double[] powers = chassis.updateSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX(), AngleUnit.normalizeRadians(follower.getPose().getHeading()+ Math.toRadians(180))  - headingOffset);
        //chassis.powerTest(driver1.getRightX());
        //telemetryA.addData("frontLeft:",powers[0]);
//        telemetryA.addData("frontRight:",powers[2]);
//        telemetryA.addData("backLeft",powers[1]);
//        telemetryA.addData("backRight",powers[3]);
        telemetryA.addData("cords:","x=%.3f y=%.3f",follower.getPose().getX(),follower.getPose().getY());
        //telemetryA.addData("distOdom:", IO.getDistanceOdom(follower.getPose()));
        //telemetryA.addData("heading: ",imu.getRobotYawPitchRollAngles().getYaw());
        //telemetryA.addData("headingFollower: ",Math.toDegrees(follower.getPose().getHeading()));
        //telemetryA.addData("headingOffset",headingOffset);
        //telemetryA.addData("sens1R:", IO.getRGB()[0]);
        //telemetryA.addData("sens1G:", IO.getRGB()[1]);
        //telemetryA.addData("sens1B:", IO.getRGB()[2]);
        //telemetryA.addData("sens1Dominant:", IO.readSensor());
        //telemetryA.addData("0",IO.ALL[0]);
        //telemetryA.addData("1:",IO.ALL[1]);
        //telemetryA.addData("2:",IO.ALL[2]);
        if (interpValues != null) {
            telemetryA.addData("    interpAngle:", interpValues[1]);
            telemetryA.addData("    interpRPM:", interpValues[0]);
        }
        telemetryA.addData("SORTER:","target=%.4f  SORTER=%.4f  error=%.4f",IO.returnTargetPos(),IO.returnSorterPos(),IO.returnTargetPos()+IO.returnSorterPos());
        telemetryA.addData("FLYWHEEL:","target=%.4f  RPM=%.4f  error=%.4f",IO.returnTargetRPM(),IO.returnRPM(),IO.returnTargetRPM()-IO.returnRPM());
        telemetryA.addData("TURRET:","target=%.4f  TURRET=%.4f  error=%.4f",IO.returnTargetTuret("ticks"),IO.returnTuret()[0],IO.returnTargetTuret("ticks")-IO.returnTuret()[0]);
        //telemetryA.addData("targetTurret", IO.rads2ticks(IO.returnTargetTuret()));
        //telemetryA.addData("turret",IO.returnTuret()[0]);
        //telemetryA.addData("turretVel", IO.returnTuret()[1]);
        telemetryA.addLine("-------test values-----");
        //telemetryA.addData("rpm",IO.returnRPM());
        //telemetryA.addData("targetRpm",IO.returnTargetRPM());
        telemetryA.addData("targetPos", IO.returnTargetPos());
        telemetryA.addData("Pos", IO.returnSorterPos());

        //telemetryA.addData("targetPos",IO.returnTargetPos());
        //telemetryA.addData("SorterPos",IO.returnSorterPos());
        //telemetryA.addData("error",IO.returnTargetPos()-IO.returnSorterPos());
        //telemetryA.addData("ReadyTurret:",IO.testTurretReady(alpha,follower));
        //telemetryA.addData("isReadyTurret:",IO.isTurretReady(alpha,follower));
        //telemetryA.addData("turretTarget",IO.returnTargetTuret("ticks"));
        //telemetryA.addData("turretPos",IO.returnTuret()[0]);

        telemetryA.update();

    }
}