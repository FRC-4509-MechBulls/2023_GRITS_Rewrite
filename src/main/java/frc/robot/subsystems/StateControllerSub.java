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



        if(oldState.armMode != desiredState.armMode){

            if(desiredState.armMode == AgArmMode.HOLDING){
                if(oldState.armMode == AgArmMode.PLACING){
                    if(desiredState.itemType == ItemType.CONE){
                        //were placing cone, now are retracting

                        switch(desiredState.placementLevel){
                            case  LEVEL3: ArmCommands.retractFromConeL3(arm).schedule(); break;
                            case LEVEL2: ArmCommands.retractFromConeL2(arm).schedule(); break;
                            case LEVEL1: ArmCommands.retractConeL1(arm).schedule(); break; //retract cone bottom
                        }

                    }

                }


            }

            if(desiredState.armMode == AgArmMode.PLACING){
                if(oldState.armMode == AgArmMode.HOLDING){
                    if(desiredState.itemType == ItemType.CONE){

                        
                        switch(desiredState.placementLevel){
                            case LEVEL3: ArmCommands.placeConeL3Example(arm).schedule(); break;
                            case LEVEL2: ArmCommands.placeConeL2Example(arm).schedule(); break;
                            case LEVEL1: ArmCommands.placeConeL1(arm).schedule(); break; //place cone bottom
                        }
                    }
                }

            }

            if(desiredState.armMode == AgArmMode.INTAKING){

            }


        }




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

