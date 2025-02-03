// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PoseConstants;


/** Add your docs here. */
public class CommandState extends SubsystemBase{
    private String m_selectedPose;
    private boolean m_isRecording = false;
    private Pose2d m_goalPose;

    public CommandState(String initalPose) {
        m_selectedPose = initalPose;
    }

    public Command toggleRecording() {
        return runOnce (
            () -> {
                m_isRecording = !m_isRecording;

                if(m_isRecording== false) {
                    switch (m_selectedPose) {
                        case "close":
                            m_goalPose = PoseConstants.ReefPosition1.getPose();
                            break;
                    
                        
                            
                    }
                }
            }
        );
    }

    public Command setSelectedPose(String direction) {
        return runOnce(
        ()-> {
            System.out.println(m_isRecording);
            if(m_isRecording){
                switch (direction) {
                    case "up" :
                        m_selectedPose = "far";
                        break;
                    case "down" :
                        m_selectedPose = "near";
                        break;
                    case "left" :
                        if(m_selectedPose.equals("far")) {
                            m_selectedPose = "farLeft";
                        }
                        else if (m_selectedPose.equals("near")) {
                            m_selectedPose = "nearLeft";
                        }
                        break;
                    case "right" :
                        if(m_selectedPose.equals("far")) {
                            m_selectedPose = "farRight";
                        }
                        else if (m_selectedPose.equals("near")) {
                            m_selectedPose = "nearRight";
                        }
                        break;    
                }
                System.out.println(m_selectedPose);
            }
           
        }
        );
    }
    public Pose2d getSelectedPose2d() {
        return m_goalPose;
    }
}
