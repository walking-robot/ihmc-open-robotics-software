package us.ihmc.commonWalkingControlModules;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

import java.io.InputStream;

import static org.junit.Assert.assertTrue;

public abstract class AvatarDynamicReachabilityCalculatorTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static String shortScript = "scripts/ExerciseAndJUnitScripts/dynamicReachabilityForwardShort.xml";
   private static String mediumScript = "scripts/ExerciseAndJUnitScripts/dynamicReachabilityForwardMedium.xml";
   private static String longScript = "scripts/ExerciseAndJUnitScripts/dynamicReachabilityForwardLong.xml";

   private static double simulationTime = 10.0;

   private double swingTime, transferTime;
   private double totalMass;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected abstract DRCRobotModel getRobotModel();

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testForwardWalkingShort() throws SimulationExceededMaximumTimeException
   {
      setupTest(shortScript);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testForwardWalkingMedium() throws SimulationExceededMaximumTimeException
   {
      setupTest(mediumScript);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testForwardWalkingLong() throws SimulationExceededMaximumTimeException
   {
      setupTest(longScript);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      assertTrue(success);
   }


   private void setupTest(String scriptName) throws SimulationExceededMaximumTimeException
   {
      this.setupTest(scriptName, ReferenceFrame.getWorldFrame());
   }

   private void setupTest(String scriptName, ReferenceFrame yawReferenceFrame) throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      totalMass = fullRobotModel.getTotalMass();
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      if (scriptName != null && !scriptName.isEmpty())
      {
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001);
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
         if (yawReferenceFrame != null)
         {
            drcSimulationTestHelper.loadScriptFile(scriptInputStream, yawReferenceFrame);
         }
         else
         {
            drcSimulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule",
               footPrefix + "State");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController",
               "walkingState");
      }

      setupCamera(scs);
      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      ThreadTools.sleep(1000);
   }

   private void setupCamera(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
