package us.ihmc.footstepPlanning.flatGroundPlanning;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.simplePlanners.FlatGroundPlanningUtils;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class FootstepPlannerOnFlatGroundTest implements PlanningTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double stepWidth = 0.3;
   private final Random random = new Random(727434726273L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJustStraightLine()
   {
      testJustStraightLine(true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJustStraightLine(boolean assertPlannerReturnedResult)
   {
      double xGoal = 5.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = 0.0;
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2D initialStanceFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialStanceFootPose = new FramePose2D(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);

      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }

   public void testATightTurn()
   {
      testJustStraightLine(true);
   }

   public void testATightTurn(boolean assertPlannerReturnedResult)
   {
      double xGoal = 1.0;
      double yGoal = 0.5;
      double yawGoal = 0.0;
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2D initialStanceFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialStanceFootPose = new FramePose2D(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
      if (assertPlannerReturnedResult) assertTrue(footstepPlan.getNumberOfSteps() < 30);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
   }

   public void testStraightLineWithInitialTurn()
   {
      testStraightLineWithInitialTurn(true);
   }

   public void testStraightLineWithInitialTurn(boolean assertPlannerReturnedResult)
   {
      double xGoal = 5.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = Math.toRadians(20.0);
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = Math.toRadians(20.0);
      Point2D initialStanceFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialStanceFootPose = new FramePose2D(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }

   public void testJustTurnInPlace()
   {
      testJustTurnInPlace(true);
   }

   public void testJustTurnInPlace(boolean assertPlannerReturnedResult)
   {
      double xGoal = 0.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = Math.toRadians(160.0);
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2D initialStanceFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialStanceFootPose = new FramePose2D(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }

   public void testRandomPoses()
   {
      testRandomPoses(true);
   }

   public void testRandomPoses(boolean assertPlannerReturnedResult)
   {
      double xGoal = random.nextDouble();
      double yGoal = random.nextDouble();
      double yawGoal = 0.0;
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = random.nextDouble();
      double yInitialStanceFoot = random.nextDouble();
      double yawInitial = 0.0;
      Point2D initialStanceFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialStanceFootPose = new FramePose2D(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.generateRandomRobotSide(random);

      FramePose3D initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }
}
