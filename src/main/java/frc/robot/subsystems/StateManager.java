package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AlgaeConstants;


public class StateManager extends SubsystemBase {
    public static class State {
        public static class Reef {
            
            public static class Coral {
                public static class Pivot {
                    public static class Position {
                        public static final int UP = 0;
                        public static final int DOWN = 1;
                    }
                }
                public static class Roller {
                    public static class Speed {
                        public static final double INTAKE = 0.5;
                        public static final double OUTTAKE = -0.5;
                        public static final double STOP = 0.0;
                    }
                }
            }
        }
    

    }

public static class Algae {
    public static class Pivot {
        public static class Position {
            public static final int UP = 0;
            public static final int DOWN = 1;
        }
    }
    public static class Roller {
        public static class Speed {
            public static final double INTAKE = 0.5;
            public static final double OUTTAKE = -0.5;
            public static final double STOP = 0.0;
        }
    }
}
   
}
    