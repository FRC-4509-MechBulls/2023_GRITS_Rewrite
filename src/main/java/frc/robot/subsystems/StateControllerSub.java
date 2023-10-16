package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands;
import frc.robot.subsystems.arm.Arm;

public class StateControllerSub extends SubsystemBase {

    Arm arm;

    public enum AgArmMode {PLACING,HOLDING,INTAKING};
    public enum ItemIsFallen {FALLEN_CONE,NOT_FALLEN};
    public enum Item{CUBE,CONE}
    public enum PlacementLevel {BOTTOM,MIDDLE,TOP};


    public AgArmMode armMode = AgArmMode.HOLDING;
    public ItemIsFallen itemIsFallen = ItemIsFallen.NOT_FALLEN;
    public Item item = Item.CUBE;
    public PlacementLevel placementLevel = PlacementLevel.BOTTOM;



    public AgArmMode oldArmMode = AgArmMode.HOLDING;
    public ItemIsFallen oldItemIsFallen = ItemIsFallen.NOT_FALLEN;
    public Item oldItem = Item.CUBE;
    public PlacementLevel oldPlacementLevel = PlacementLevel.BOTTOM;





    public StateControllerSub(Arm arm) {
        this.arm = arm;
    }



    public void periodic(){

        SmartDashboard.putString("armMode",armMode.toString());
        SmartDashboard.putString("itemIsFallen",itemIsFallen.toString());
        SmartDashboard.putString("itemType",item.toString());
        SmartDashboard.putString("placementLevel",placementLevel.toString());


        if(oldArmMode == armMode && oldItemIsFallen == itemIsFallen && placementLevel == oldPlacementLevel && oldItem == item){
            return;
        }


        if(oldArmMode != armMode){

            if(armMode == AgArmMode.HOLDING){
                if(oldArmMode == AgArmMode.PLACING){
                    if(item == Item.CONE){
                        //were placing cone, now are retracting

                        
                        switch(placementLevel){
                            case  TOP: ArmCommands.retractFromConeL3(arm).schedule(); break;
                            case MIDDLE: ArmCommands.retractFromConeL2(arm).schedule(); break;
                            case BOTTOM: ArmCommands.retractConeL1(arm); break; //retract cone bottom
                        }

                    }

                }


            }

            if(armMode == AgArmMode.PLACING){
                if(oldArmMode == AgArmMode.HOLDING){
                    if(item == Item.CONE){

                        
                        switch(placementLevel){
                            case TOP: ArmCommands.placeConeL3Example(arm).schedule(); break;
                            case MIDDLE: ArmCommands.placeConeL2Example(arm).schedule(); break;
                            case BOTTOM: ArmCommands.placeConeL1(arm); break; //place cone bottom
                        }
                    }
                }

            }

            if(armMode == AgArmMode.INTAKING){

            }


        }





        oldArmMode = armMode;
        oldItemIsFallen = itemIsFallen;
        oldItem = item;
        oldPlacementLevel = placementLevel;


    }

    void placeholder(){};

    public void setArmModeToHolding() {
        armMode = AgArmMode.HOLDING;
    }
    public void setArmModeToIntaking() {
        armMode = AgArmMode.INTAKING;
    }
    public void setArmModeToPlacing() {
        armMode = AgArmMode.PLACING;
    }



    public void setArmLevelBottom(){
       placementLevel = PlacementLevel.BOTTOM;
    }

    public void setArmLevelMiddle(){
        placementLevel = PlacementLevel.MIDDLE;
    }

    public void setArmLevelTop(){
        placementLevel = PlacementLevel.TOP;
    }



    public void setItemConeFallen(){
        item = Item.CONE;
        itemIsFallen = ItemIsFallen.FALLEN_CONE;
    }
    public void setItemConeUpright() {
        item = Item.CONE;
        itemIsFallen = ItemIsFallen.NOT_FALLEN;
    }



    public void setItemCubeFallen(){
        item = Item.CUBE;
        itemIsFallen = ItemIsFallen.NOT_FALLEN;
    }


/**
    public void invokeSomethingFun(){
       ArmCommands.placeConeL3Example(arm).schedule();
    }
*/
}

