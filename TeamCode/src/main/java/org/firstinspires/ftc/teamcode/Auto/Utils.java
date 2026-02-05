package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;

public class Utils {
    Follower follower;
    IOSubsystem IO;
    public Utils(IOSubsystem IO, Follower follower){
        this.IO = IO;
        this.follower = follower;
    }
    boolean succesDemand;
    double alpha;

    public Command newAutoOutake(int timeout){
        Command quickPush = new SequentialCommandGroup(
                new InstantCommand(() -> IO.setPush(IO.PUSH_MAX_LIMIT)),
                new WaitCommand(35),
                new InstantCommand(() -> IO.setPush(IO.PUSH_MIN_LIMIT)),
                new WaitCommand(35),
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
            double curAlpha = AngleUnit.normalizeRadians(follower.getPose().getHeading());
            IO.setTargetTurretRads((AngleUnit.normalizeRadians(curAlpha - alpha)));
        });

        Command stopAutoOutake = new InstantCommand(() -> {
            IO.setMotorRPM(0);
            IO.setHood(0.1);
            IO.setTargetTurretRads(0);
            IO.setPush(IO.PUSH_MIN_LIMIT);
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


    public Command newStartIntake(int timeout){
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
                                        new WaitCommand(20),   // do nothing
                                        () -> (IO.isOcupied()) // if recived ball
                                ))

                        ).withTimeout(timeout),
                        stopIntake
                );

    }



}
