package us.ihmc.exampleSimulations.simple3DWalkerTest;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.exampleSimulations.simple3DWalker.SimpleWalkerHeightStopMPC;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleWalkerHeightForStopMPCTest
{
   YoVariableRegistry registry = new YoVariableRegistry("registry");
   double umax = 1000;
   double zmax = 1.1;
   double zf = 1.0;
   double x;

   SimpleWalkerHeightStopMPC heightMPC = new SimpleWalkerHeightStopMPC(zmax, zf, umax, registry);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMaxHeightMaxUOutLoop()
   {
      for (int i = 0; i < 10; i++)
      {
         x = -0.3193 + i * 0.02;
         heightMPC.computeInvOutLoop(x, 1.0, 1.0, 0.0);
         double uinit = heightMPC.getU();
         Assert.assertTrue("U exceeds defined max", uinit <= umax);
         double zMaxProg = heightMPC.getZmaxComputed();
         Assert.assertTrue("Max height of polynomial exceeds requested", zMaxProg <= zmax);
         // double zMaxPol =
         if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         {
            PrintTools.info("The algorithm took " + heightMPC.getIters() + " iterations and " + heightMPC.getComputationTimems() + " ns and dxf= " + heightMPC
                  .getDxf() + " polcst " + heightMPC.getC()[0] + " " + heightMPC.getC()[1] + " " + heightMPC.getC()[2] + " " + heightMPC.getC()[3] + " ");
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMaxHeightMaxUInLoop()
   {
      for (int i = 0; i < 10; i++)
      {
         x = -0.3193 + i * 0.02;
         heightMPC.computeInvInLoop(x, 1.0, 1.0, 0.0);
         double uinit = heightMPC.getU();
         Assert.assertTrue("U exceeds defined max", uinit <= umax);
         double zMaxProg = heightMPC.getZmaxComputed();
         Assert.assertTrue("Max height of polynomial exceeds requested", zMaxProg <= zmax);
         // double zMaxPol =
         if(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         {
            PrintTools.info("The algorithm took " + heightMPC.getIters() + " iterations and " + heightMPC.getComputationTimems() + " ns and dxf= " + heightMPC.getDxf()
                                  + " polcst " + heightMPC.getC()[0] + " " + heightMPC.getC()[1] + " " + heightMPC.getC()[2] + " " + heightMPC.getC()[3] + " ");
         }
      }
   }
}
