package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands;
import frc.robot.subsystems.arm.Arm;

public class StateControllerSub extends SubsystemBase {

    Arm arm;
    EndEffectorSub ef;

    public enum AgArmMode {PLACING,POST_PLACING,HOLDING,INTAKING};
    public enum ItemIsFallen {FALLEN_CONE,NOT_FALLEN};
    public enum ItemType {CUBE,CONE}
    public enum PlacementLevel {LEVEL1,LEVEL2,LEVEL3};

    ArmState desiredState = new ArmState();
    ArmState oldState = new ArmState();




    public StateControllerSub(Arm arm, EndEffectorSub ef) {
        this.arm = arm;
        this.ef = ef;
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
                    switch (desiredState.itemType){
                        case CONE:ef.holdCone(); break;
                        case CUBE:ef.holdCube(); break;
                    }
                    switch (oldState.armMode) {
                        case PLACING: case POST_PLACING:
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
                            if(desiredState.itemType == ItemType.CUBE){
                                switch (oldState.placementLevel) {
                                    case LEVEL3:
                                    case LEVEL2:
                                        ArmCommands.retractCubeFromL2orL3(arm).schedule();
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
                            if(desiredState.itemType == ItemType.CUBE) {
                                ArmCommands.quickHolding(arm).schedule();
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

                    if(oldState.armMode == AgArmMode.HOLDING && desiredState.itemType == ItemType.CUBE){
                        switch (desiredState.placementLevel){
                            case LEVEL2:
                            case LEVEL3:
                                ArmCommands.placeCubeL2orL3(arm).schedule();
                                break;
                            case LEVEL1:
                                ArmCommands.placeCubeL1(arm).schedule();
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
                            ef.intakeCone();
                            terminate();
                        } else if (desiredState.itemType == ItemType.CUBE) {
                            // cube gangsta mode
                            ArmCommands.intakeCube(arm).schedule();
                            ef.intakeCube();
                            terminate();
                        }
                    }
                    break;

                case POST_PLACING:
                    switch (desiredState.itemType){
                        case CONE: ef.placeCone(); break;
                        case CUBE:
                            switch(desiredState.placementLevel){
                                case LEVEL3:
                                case LEVEL1:
                                    ef.placeCubeTop(); break;
                                case LEVEL2: ef.placeCubeBottom(); break;
                            }
                            break;
                    }
                    terminate();
                    break;
            }
        }
        if(desiredState.itemType!= oldState.itemType){ //TODO: maybe remove this?
            switch(desiredState.itemType){
                case CONE: ef.holdCone(); break;
                case CUBE: ef.holdCube(); break;
            }
            oldState.itemType = desiredState.itemType; //remove this too
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
    public void setArmModeToPostPlacing() {
        if(desiredState.armMode != AgArmMode.PLACING) return;
        desiredState.armMode = AgArmMode.POST_PLACING;
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

    public void setOverallStateSafe(ArmState state){
        desiredState.itemType = state.itemType;
        periodic();
        desiredState.itemIsFallen = state.itemIsFallen;
        periodic();
        desiredState.placementLevel = state.placementLevel;
        periodic();
        desiredState.armMode = state.armMode;
        periodic();
    }



    public void setItemConeFallen(){
        desiredState.itemType = ItemType.CONE;
        desiredState.itemIsFallen = ItemIsFallen.FALLEN_CONE;
    }
    public void setItemConeUpright() {
        desiredState.itemType = ItemType.CONE;
        desiredState.itemIsFallen = ItemIsFallen.NOT_FALLEN;
    }

    
    public void setItemCube(){
        desiredState.itemType = ItemType.CUBE;
        desiredState.itemIsFallen = ItemIsFallen.NOT_FALLEN;
    }

    public void overrideEFStop(){
        ef.stopMotors();
    }


/**
    public void invokeSomethingFun(){
       ArmCommands.placeConeL3Example(arm).schedule();
    }
*/
}


