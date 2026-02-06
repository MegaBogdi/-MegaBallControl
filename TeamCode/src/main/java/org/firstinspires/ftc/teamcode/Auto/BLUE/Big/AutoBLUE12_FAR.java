package org.firstinspires.ftc.teamcode.Auto.BLUE.Big;

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
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auto.Utils;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "AutonomieBLUE_12_FAR", group = "Auto Blue")
public class AutoBLUE12_FAR extends CommandOpMode {
    private Pose startPose = new Pose(57.018691588785046,6.857009345794408,Math.toRadians(180));
    private Pose spike3Pos = new Pose(11.558878504672895,35.35700934579436,Math.toRadians(180));
    private Pose helpSpike3Pos = new Pose(58.43831775700934, 41.42476635514023,Math.toRadians(180));
    private Pose shootFarPos =  new Pose(57.14953271028037, 14.17757009345793,Math.toRadians(180));
    private Pose spike2Pos = new Pose(17.23364485981309,64.34579439252336,Math.toRadians(180));
    private Pose helpSpike2Pos = new Pose(54.8271028037383,80.63551401869161,Math.toRadians(180));
    private Pose HelpSpike2Pos2 = new Pose(34.81775700934578,48.271028037383175,Math.toRadians(180));
    private Pose shootShortPos = new Pose(48.67289719626168,84.86915887850468,Math.toRadians(180));
    private Pose spike1Pos = new Pose(18.476635514018692,84.11214953271028,Math.toRadians(180));
    private Pose parkPos = new Pose(47.85981308411215,71.53271028037383,Math.toRadians(90));
    //private Pose parkSafePos= new Pose(35.36448598130842,59.32710280373832,toRad180(90));

    private PathChain StartToSpike3Path, Spike3ToShootFarPath, ShootFarToSpike2LeverPath,LeverToShootShortPath,ShootShortToSpike1Path,Spike1ToShootShortPath,ShootShortToParkPath;
    private Telemetry telemetryA;
    private IOSubsystem IO;
    private Follower follower;

    private Utils utils;
    private double alpha;
    private boolean succesDemand;
    private int id;

    private int autoOutakeTimeout = 5000;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.setMaxPower(1);
        IO = new IOSubsystem(hardwareMap);
        register(IO);
        IO.startLimeLight();
        utils = new Utils(IO,follower);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = telemetryA;
        IO.teamIsRed = false;

        StartToSpike3Path = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,helpSpike3Pos,spike3Pos))
                .setConstantHeadingInterpolation(shootFarPos.getHeading())
                .build();
        Spike3ToShootFarPath = follower.pathBuilder()
                .addPath(new BezierLine(spike3Pos,shootFarPos))
                .setConstantHeadingInterpolation(shootFarPos.getHeading())
                .build();
        ShootFarToSpike2LeverPath = follower.pathBuilder()
                .addPath(new BezierCurve(shootFarPos,helpSpike2Pos,HelpSpike2Pos2,spike2Pos))
                .setConstantHeadingInterpolation(spike2Pos.getHeading())
                .build();
        LeverToShootShortPath = follower.pathBuilder()
                .addPath(new BezierLine(spike2Pos,shootShortPos))
                .setConstantHeadingInterpolation(shootShortPos.getHeading())
                .build();
        ShootShortToSpike1Path = follower.pathBuilder()
                .addPath(new BezierLine(shootShortPos,spike1Pos))
                .setConstantHeadingInterpolation(spike1Pos.getHeading())
                .build();

        Spike1ToShootShortPath = follower.pathBuilder()
                .addPath(new BezierLine(spike1Pos,shootShortPos))
                .setConstantHeadingInterpolation(shootShortPos.getHeading())
                .build();
        ShootShortToParkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootShortPos,parkPos))
                .setLinearHeadingInterpolation(shootShortPos.getHeading(),parkPos.getHeading())
                .build();


        schedule(
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(()->{IO.ALL[0]=1;IO.ALL[1]=2;IO.ALL[2]=1;}), // SET SPECIFIC BALL START POSITION
                        //newStartIntake(4000).andThen(newStopIntake()).alongWith(new SequentialCommandGroup(new InstantCommand(()->IO.setTargetTurretRads(-1.5)),newMotify(3000))),   // READ BALLS IN SORTER WHILE GRABING MOTIF
                        new ParallelCommandGroup(  //GRAB MOTIF
                                newMotify(1000),
                                new InstantCommand(()->IO.setTargetTurretRads(Math.toRadians(-90))) //!!!!!!!!!!!!!!!!! REVERSE if wrong
                        ),
                        utils.newAutoOutake(4500), //PRELOAD   //GANDESTE LA UN RPM MEDIU CONSTANT PT CONSERVARE  (FRIGIDER)
                        new FollowPathCommand(follower, StartToSpike3Path).beforeStarting(()->follower.setMaxPower(0.8)).alongWith(utils.newStartIntake(3500)).andThen(newStopIntake()),
                        new FollowPathCommand(follower,Spike3ToShootFarPath).beforeStarting(()->follower.setMaxPower(1)),
                        utils.newAutoOutake(4500),//SPIKE 3 (FAR)
                        new FollowPathCommand(follower, ShootFarToSpike2LeverPath).alongWith(utils.newStartIntake(3000)).beforeStarting(()->follower.setMaxPower(0.7)),
                        new WaitCommand(500), // HOW LONG WE WAIT FOR BALLS TO FLOW:


                        new FollowPathCommand(follower, LeverToShootShortPath).beforeStarting(()->follower.setMaxPower(1)),
                        utils.newAutoOutake(3000), // SPIKE2 (SHORT)

                        new FollowPathCommand(follower, ShootShortToSpike1Path).beforeStarting(()->follower.setMaxPower(0.8)).alongWith(utils.newStartIntake(5000)).andThen(newStopIntake()),
                        new FollowPathCommand(follower, Spike1ToShootShortPath).beforeStarting(()->follower.setMaxPower(1)),
                        utils.newAutoOutake(3000),

                        new FollowPathCommand(follower, ShootShortToParkPath).beforeStarting(()->follower.setMaxPower(1)),
                        new WaitCommand(30000)

                )
        );


    }

    @Override
    public void run() {
        super.run();              // runs CommandScheduler
        follower.update();        // keep follower alive

        Pose p = follower.getPose();
        telemetryA.addData("dist",IO.getDistanceOdom(follower.getPose()));
        telemetryA.addData("coord", "x=%.2f  y=%.2f  h=%.1f", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        //telemetry.addData("sug coaie:",1);
        telemetryA.addData("ALL: ","0=%d | 1=%d | 2=%d",IO.ALL[0],IO.ALL[1],IO.ALL[2]);
        telemetryA.addData("condition:","RPM:%b | turret:%b,sorter:%b",IO.isRPMready(),IO.isTurretReady(alpha,follower),IO.isSorterReady());
        telemetryA.addData("rpm",IO.returnRPM());
        telemetryA.addData("targetRpm",IO.returnTargetRPM());
        telemetryA.addData("motify",IO.getMotif());
        telemetry.update();

        IO.update_sep_pid();
        IO.update_RPM_pid();
        IO.update_turret_pid();
    }

    public double toRad180(double degrees){
        return AngleUnit.normalizeRadians(Math.toRadians(degrees)+Math.toRadians(180));
    }

    private Command newAutoOutake_______(int timeout){
        Command quickPush = new SequentialCommandGroup(
                new InstantCommand(() -> IO.setPush(0.3)),
                new WaitCommand(200),
                new InstantCommand(() -> IO.setPush(0)),
                new WaitCommand(200),
                new InstantCommand(() -> IO.regist_release())
        );
        Command Demand = new SequentialCommandGroup(
                new InstantCommand(() -> succesDemand = IO.getDemanded()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> (IO.isSorterReady() && IO.isRPMready() && IO.isTurretReady(alpha, follower))),
                                quickPush
                        ),
                        new InstantCommand(() -> IO.demand_index += 1),
                        () -> succesDemand
                )
        );
        Command interp = new InstantCommand(() -> {
            double dist = IO.getDistanceOdom(follower.getPose());
            double[] interpValues = IO.getInterpolatedValues(dist);
            IO.setHood(interpValues[1]);
            IO.setMotorRPM(interpValues[0]);
        });
        Command autoAim = new InstantCommand(() -> {
            alpha = IO.getAngle(follower.getPose());
            double curAlpha = AngleUnit.normalizeRadians(follower.getPose().getHeading() + Math.toRadians(180)); //facem asta pentru ca fata robotului real e defapt spatele robotului virtual
            IO.setTargetTurretRads(-(AngleUnit.normalizeRadians(curAlpha - alpha)));
        });

        Command stopAutoOutake = new InstantCommand(() -> {
            IO.setMotorRPM(0);
            IO.setHood(0.1);
            IO.setTargetTurretRads(0);
            IO.setPush(0);
        });                   // complete?
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> IO.ocupied() == 0),  //shoot until empty
                        new RepeatCommand(interp),
                        new RepeatCommand(autoAim),
                        new RepeatCommand(Demand)
                ).withTimeout(timeout),
                stopAutoOutake
        );

    }

    private Command newStartIntake__(int timeout){
        Command stopIntake = new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });

        return
                new SequentialCommandGroup(
                        new InstantCommand(()->IO.open()),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()->IO.ocupied() >= 3),  // while not full
                                new RunCommand(()->IO.start_intake()),
                                new RepeatCommand(new ConditionalCommand( // if ball incoming
                                        new SequentialCommandGroup(
                                                new WaitCommand(50),
                                                new InstantCommand(() -> IO.regist()),// register the incoming ball in memory
                                                new InstantCommand(() -> IO.climb()), // phisicly spin the sorter
                                                new WaitUntilCommand(() -> !IO.isOcupied())
                                        ),
                                        new SequentialCommandGroup(new InstantCommand(()->{telemetryA.addLine("detecting...");telemetryA.update();}),new WaitCommand(20)),   // do nothing
                                        () -> (IO.isOcupied()) // if recived ball
                                ))

                        ).withTimeout(timeout),
                        stopIntake
                );

    }

    private Command newStopIntake(){
        return new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });
    }

    private Command newMotify(int timeout){
        return
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()->IO.checkMotifFind(id)),
                                new RunCommand(()-> id =IO.getMotif())
                        ).withTimeout(timeout),
                        new InstantCommand(()->{IO.motifTranslate(id);IO.stopLimeLight();})
                );

    }




}

/* uai deci:

* nu e bun demand intexu pentru ca isi ia offset cand nu are ce ii trebuie
-fa un demand_index_offset




*/