package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands;
import frc.robot.subsystems.arm.Arm;

public class StateControllerSub extends SubsystemBase {

    Arm arm;

    public enum AgArmMode {PLACING,HOLDING,INTAKING};
    public enum ItemIsFallen {FALLEN_CONE,NOT_FALLEN};
    public enum ItemType {CUBE,CONE}
    public enum PlacementLevel {LEVEL1,LEVEL2,LEVEL3};

    ArmState desiredState = new ArmState();
    ArmState oldState = new ArmState();




    public StateControllerSub(Arm arm) {
        this.arm = arm;
    }



    public void periodic(){

        SmartDashboard.putString("armMode",desiredState.armMode.toString());
        SmartDashboard.putString("itemIsFallen",desiredState.itemIsFallen.toString());
        SmartDashboard.putString("itemType",desiredState.itemType.toString());
        SmartDashboard.putString("placementLevel",desiredState.placementLevel.toString());


        if(desiredState.equals(oldState)){
            return;
        }



        if (oldState.armMode != desiredState.armMode) {
            switch (desiredState.armMode) {
                case HOLDING:
                    switch (oldState.armMode) {
                        case PLACING:
                            if (desiredState.itemType == ItemType.CONE) {
                                switch (oldState.placementLevel) {
                                    case LEVEL3:
                                        ArmCommands.retractFromConeL3(arm).schedule();
                                        break;
                                    case LEVEL2:
                                        ArmCommands.retractFromConeL2(arm).schedule();
                                        break;
                                    case LEVEL1:
                                        ArmCommands.quickHolding(arm).schedule(); // retract cone bottom
                                        break;
                                }
                                terminate();
                            }
                            break;

                        case INTAKING:
                            if (desiredState.itemType == ItemType.CONE) {
                                if(desiredState.itemIsFallen == ItemIsFallen.NOT_FALLEN)
                                    ArmCommands.quickHolding(arm).schedule();
                                else
                                    ArmCommands.retractFromConeFallen(arm).schedule();
                                terminate();
                            }
                            break;
                    }
                    break;

                case PLACING:
                    if (oldState.armMode == AgArmMode.HOLDING && desiredState.itemType == ItemType.CONE) {
                        switch (desiredState.placementLevel) {
                            case LEVEL3:
                                ArmCommands.placeConeL3(arm).schedule();
                                break;
                            case LEVEL2:
                                ArmCommands.placeConeL2(arm).schedule();
                                break;
                            case LEVEL1:
                                ArmCommands.placeConeL1(arm).schedule(); // place cone bottom
                                break;
                        }
                        terminate();
                    }
                    break;

                case INTAKING:
                    if (oldState.armMode == AgArmMode.HOLDING) {
                        if (desiredState.itemType == ItemType.CONE) {
                            switch (desiredState.itemIsFallen) {
                                case FALLEN_CONE:
                                    ArmCommands.intakeConeFallen(arm).schedule();
                                    break;
                                case NOT_FALLEN:
                                    ArmCommands.intakeConeUpright(arm).schedule();
                                    break;
                            }
                            terminate();
                        } else if (desiredState.itemType == ItemType.CUBE) {
                            // cube gangsta mode
                            // your logic here
                        }
                    }
                    break;
            }
        }


    }
    private void terminate(){
        oldState = new ArmState(desiredState.armMode,desiredState.itemType,desiredState.itemIsFallen,desiredState.placementLevel);
    }

    void placeholder(){};

    public void setArmModeToHolding() {
        desiredState.armMode = AgArmMode.HOLDING;
    }
    public void setArmModeToIntaking() {
        desiredState.armMode = AgArmMode.INTAKING;
    }
    public void setArmModeToPlacing() {
        desiredState.armMode = AgArmMode.PLACING;
    }



    public void setArmLevelBottom(){
        desiredState.placementLevel = PlacementLevel.LEVEL1;
    }

    public void setArmLevelMiddle(){
        desiredState.placementLevel = PlacementLevel.LEVEL2;
    }

    public void setArmLevelTop(){
        desiredState.placementLevel = PlacementLevel.LEVEL3;
    }



    public void setItemConeFallen(){
        desiredState.itemType = ItemType.CONE;
        desiredState.itemIsFallen = ItemIsFallen.FALLEN_CONE;
    }
    public void setItemConeUpright() {
        desiredState.itemType = ItemType.CONE;
        desiredState.itemIsFallen = ItemIsFallen.NOT_FALLEN;
    }



    public void setItemCubeFallen(){
        desiredState.itemType = ItemType.CUBE;
        desiredState.itemIsFallen = ItemIsFallen.NOT_FALLEN;
    }


/**
    public void invokeSomethingFun(){
       ArmCommands.placeConeL3Example(arm).schedule();
    }
*/
}

