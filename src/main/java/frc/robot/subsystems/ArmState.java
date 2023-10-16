package frc.robot.subsystems;
import static frc.robot.subsystems.StateControllerSub.*;

public class ArmState {
   public AgArmMode armMode = AgArmMode.HOLDING;
   public ItemType itemType = ItemType.CUBE;
   public ItemIsFallen itemIsFallen = ItemIsFallen.NOT_FALLEN;
   public PlacementLevel placementLevel = PlacementLevel.LEVEL1;


    public ArmState(AgArmMode armMode, ItemType itemType, ItemIsFallen itemIsFallen, PlacementLevel placementLevel){
        this.armMode = armMode;
        this.itemType = itemType;
        this.itemIsFallen = itemIsFallen;
        this.placementLevel = placementLevel;
    }

    public ArmState(){

    }

    public boolean equals(ArmState other){
        return this.armMode == other.armMode && this.itemType == other.itemType && this.itemIsFallen == other.itemIsFallen && this.placementLevel == other.placementLevel;
    }



}
