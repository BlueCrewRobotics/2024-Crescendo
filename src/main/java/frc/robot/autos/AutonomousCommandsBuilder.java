package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootNoteIntoAmp;
import frc.robot.commands.ShootNoteIntoSpeaker;

import java.util.Objects;

import static frc.robot.Constants.AutoConstants.*;

public class AutonomousCommandsBuilder extends SequentialCommandGroup {

    public AutonomousCommandsBuilder(int numOfAutoActions, int numOfAmpScores, String autoLane,
                                     int numOfNotesFromStart, int searchDirection, boolean grabFromCenterFirst){

        if(numOfNotesFromStart >= numOfAutoActions) numOfNotesFromStart = numOfAutoActions-1;
        int numOfNotesFromCenter = numOfAutoActions-(numOfNotesFromStart+1);

        int[] orderOfStartNotes = orderOfNotes(numOfNotesFromStart, autoLane, searchDirection, 3);
        int[] orderOfCenterNotes = orderOfNotes(numOfNotesFromCenter, autoLane, searchDirection, 5);

        int grabsFromStartAttempted = 0;
        String lastScoredIn;
        if (numOfAutoActions > 0) {
            addCommands(/*AutoBuilder.buildAuto*/Commands.print("ShootIntoSpeaker"));
            System.out.println("Shoot Speaker");
            lastScoredIn = "Sp";
            for (int i = 0; i < numOfAutoActions-1; i++) {
                if(numOfAutoActions > 1) {
                    if (((grabFromCenterFirst && i < numOfNotesFromCenter) || (!grabFromCenterFirst && i >= numOfNotesFromStart))) {
                        System.out.println("Grab From Center");
                        addCommands(new AutoGrabFromCenter(orderOfCenterNotes, lastScoredIn, autoLane));
                        if (i < numOfAmpScores) {
                            lastScoredIn = "Amp";
                            addCommands(
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-Amp"), pathConstraints),
                                    Commands.print("Path Find To and Following: CL-" + autoLane + "-Amp"),
                                    new ShootNoteIntoAmp()
                            );
                        } else {
                            lastScoredIn = "Sp";
                            addCommands(
                                    // TODO: add command for calculating direction to face and shooter velocity, and spin up the shooter
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-Sp"), pathConstraints),
                                    Commands.print("Path Find To and Following: CL-" + autoLane + "-Sp"),
                                    new ShootNoteIntoSpeaker()
                            );
                        }
                    } else {
                        System.out.println("Grabbing From Start");
                        addCommands(new AutoGrabFromStart(orderOfStartNotes[grabsFromStartAttempted], autoLane));
                        if (i < numOfAmpScores) {
                            lastScoredIn = "Amp";
                            System.out.println("Shoot Amp");
                            addCommands(
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("SL-" + autoLane + "-Amp"), pathConstraints),
                                    Commands.print("Path Find To and Following: SL-" + autoLane + "-Amp"),
                                    new ShootNoteIntoAmp()
                            );
                        } else {
                            lastScoredIn = "Sp";
                            System.out.println("Shoot Speaker");
                            addCommands(
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("SL-" + autoLane + "-Sp"), pathConstraints),
                                    Commands.print("Path Find To and Following: SL-" + autoLane + "-Sp"),
                                    new ShootNoteIntoSpeaker()
                            );
                        }
                        grabsFromStartAttempted++;
                    }
                } else {
                    addCommands(
                            //AutoBuilder.followPath(PathPlannerPath.fromPathFile(lastScoredIn + "-" + autoLane + "-SL"))
                            Commands.print("Following: " + lastScoredIn + "-" + autoLane + "-SL")
                    );
                }
            }
        }
    }

    private int[] orderOfNotes(int numOfNotesToGet, String autoLane, int searchDirection, int totalNumOfNotes) {
        int[] order = new int[numOfNotesToGet];

        if(numOfNotesToGet < 1) return  order;
        else if(numOfNotesToGet > totalNumOfNotes) return order;

        else if(Objects.equals(autoLane, ampLane)) {
            for(int i = 0; i < numOfNotesToGet; i++) {
                order[i] = i+1;
            }
        }

        else if(Objects.equals(autoLane, sourceLane)) {
            for(int i = 0; i < numOfNotesToGet; i++) {
                order[i] = totalNumOfNotes-i;
            }
        }

        else if (Objects.equals(autoLane, stageLane)) {
            int noteNumber = (int) Math.ceil((double) totalNumOfNotes / 2);
            for(int i = 0; i < numOfNotesToGet; i++) {
                order[i] = noteNumber;

                noteNumber += searchDirection;

                if(noteNumber < 1) noteNumber += totalNumOfNotes;
                else if(noteNumber > totalNumOfNotes) noteNumber -= totalNumOfNotes;
            }
        }

        return order;
    }
}