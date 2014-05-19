package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseRampsTest;

public class ValkyrieObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{

   private DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }
   @Override
   protected double getMaxRotationCorruption()
   {
      return 0.0;
   }

}
