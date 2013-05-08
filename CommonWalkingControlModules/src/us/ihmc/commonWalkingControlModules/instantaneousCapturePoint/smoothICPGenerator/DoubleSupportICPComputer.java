package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class DoubleSupportICPComputer
{
   private FramePoint tempFramePointI = new FramePoint();
   private FramePoint tempFramePointIminus1 = new FramePoint();

   private final DenseMatrix64F doubleSupportParameterMatrix = new DenseMatrix64F(3, 4);

   private final DenseMatrix64F desiredDCMposOfTime = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F desiredDCMvelOfTime = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F desiredECMPofTime = new DenseMatrix64F(3, 1);

   private final DenseMatrix64F initialDoubleSupportICPpos = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F initialDoubleSupportICPvel = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F finalDoubleSupportICPpos = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F finalDoubleSupportICPvel = new DenseMatrix64F(3, 1);


   private boolean finalStepReached;
   private int finalCounter;


   public DoubleSupportICPComputer(YoVariableRegistry registryExt)
   {
//    tempFramePointI.set("pointTemp1","",ReferenceFrame.getWorldFrame(), registry);
//    tempFramePointIminus1 =  new YoFramePoint("pointTemp2","",ReferenceFrame.getWorldFrame(), registry);

      finalStepReached = false;
      finalCounter = 0;

   }

   public DenseMatrix64F getInitialDoubleSupportICPpos()
   {
      return initialDoubleSupportICPpos;
   }

// public DenseMatrix64F getInitialDoubleSupportICPvel()
// {
//    return initialDoubleSupportICPvel;
// }

   public DenseMatrix64F getFinalDoubleSupportICPpos()
   {
      return finalDoubleSupportICPpos;
   }

// public DenseMatrix64F getFinalDoubleSupportICPvel()
// {
//    return finalDoubleSupportICPvel;
// }

   public DenseMatrix64F getDesiredDCMposOfTime()
   {
      return desiredDCMposOfTime;
   }

   public DenseMatrix64F getDesiredDCMvelOfTime()
   {
      return desiredDCMvelOfTime;
   }

   public DenseMatrix64F getDesiredECMPofTime()
   {
      return desiredECMPofTime;
   }

// public DenseMatrix64F getPolynomialParamMatrix()
// {
//    return paramMatrix;
// }

// public DenseMatrix64F getPolynomialParamMatrixColumn(int colIndex)
// {
//    return EnhancedMatrixManipulator.getMatrixColumn(colIndex, paramMatrix);
// }

   public void reset()
   {
      finalStepReached = false;
      finalCounter = 0;

   }

   private void updateDCMCornerPoints(ArrayList<DenseMatrix64F> constantEquivalentCoPs, double dcmConst, double steppingTime,
                                      ArrayList<DenseMatrix64F> initialICPsToPack)
   {
      int initialICPsSize = initialICPsToPack.size();

      JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(initialICPsSize - 1), -steppingTime, dcmConst,
              constantEquivalentCoPs.get(initialICPsSize), initialICPsToPack.get(initialICPsSize - 1));

      for (int i = initialICPsToPack.size() - 1; i > 0; i--)
      {
         JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(i - 1), -steppingTime, dcmConst, initialICPsToPack.get(i), initialICPsToPack.get(i - 1));
      }
   }


   public void computeDoubleSupportPolynomialParams(ArrayList<DenseMatrix64F> constantEquivalentCoPs, ArrayList<YoFramePoint> consideredFootStepLocationsFramePoints, double dcmConst, double steppingTime,
           double doubleSupportFirstStepFraction, ArrayList<DenseMatrix64F> initialICPs, boolean isFirstStep, double initialTransferSupportTime,
           double doubleSupportTime)
   {
      double currentDoubleSupportTime;

      if (isFirstStep)
      {
         currentDoubleSupportTime = initialTransferSupportTime;
      }
      else
      {
         currentDoubleSupportTime = doubleSupportTime;
      }


      double doubleSupportTimeCurrentStep = -doubleSupportFirstStepFraction * currentDoubleSupportTime;
      double doubleSupportTimeNextStep = (1 - doubleSupportFirstStepFraction) * doubleSupportTime;
      

      updateDCMCornerPoints(constantEquivalentCoPs, dcmConst, steppingTime, initialICPs);

      // Calculate DCM position and velocity at beginning of Double Support phase

      if (isFirstStep)    // (false) //
      {
//         FramePoint tempFramePointI = new FramePoint(ReferenceFrame.getWorldFrame());
//         FramePoint tempFramePointIminus1 = new FramePoint(ReferenceFrame.getWorldFrame());
//
//
//         tempFramePointI.set(constantEquivalentCoPs.get(0).get(0), constantEquivalentCoPs.get(0).get(1), constantEquivalentCoPs.get(0).get(2));
//
//         tempFramePointIminus1.set(constantEquivalentCoPs.get(1).get(0), constantEquivalentCoPs.get(1).get(1), constantEquivalentCoPs.get(1).get(2));
//         tempFramePointI.add(tempFramePointIminus1);
//         tempFramePointI.scale(0.5);
         
         
         FramePoint tempFramePointI =  new FramePoint(ReferenceFrame.getWorldFrame());
         FramePoint tempFramePointIminus1 =  new FramePoint(ReferenceFrame.getWorldFrame());
         
         
         tempFramePointI.set(consideredFootStepLocationsFramePoints.get(0).getX(), consideredFootStepLocationsFramePoints.get(0).getY(), consideredFootStepLocationsFramePoints.get(0).getZ());  
         
         tempFramePointIminus1.set(consideredFootStepLocationsFramePoints.get(1).getX(), consideredFootStepLocationsFramePoints.get(1).getY(), consideredFootStepLocationsFramePoints.get(1).getZ());
         tempFramePointI.add(tempFramePointIminus1);
         tempFramePointI.scale(0.5);

         initialDoubleSupportICPpos.set(0, tempFramePointI.getX());
         initialDoubleSupportICPpos.set(1, tempFramePointI.getY());
         initialDoubleSupportICPpos.set(2, tempFramePointI.getZ());

         initialDoubleSupportICPvel.set(0, 0);
         initialDoubleSupportICPvel.set(1, 0);
         initialDoubleSupportICPvel.set(2, 0);
      }
      else
      {
         JojosICPutilities.extrapolateDCMposAndVel(constantEquivalentCoPs.get(0), steppingTime + doubleSupportTimeCurrentStep, dcmConst, initialICPs.get(0),
                 initialDoubleSupportICPpos, initialDoubleSupportICPvel);
      }



      // Calculate DCM position and velocity at end of Double Support phase

      JojosICPutilities.extrapolateDCMposAndVel(constantEquivalentCoPs.get(1), doubleSupportTimeNextStep, dcmConst, initialICPs.get(1),
            finalDoubleSupportICPpos, finalDoubleSupportICPvel);

      computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, currentDoubleSupportTime, initialDoubleSupportICPpos, initialDoubleSupportICPvel, finalDoubleSupportICPpos, finalDoubleSupportICPvel);
   }
   
   
   private static void computeThirdOrderPolynomialParameterMatrix(DenseMatrix64F parameterMatrixToPack, double doubleSupportTime, DenseMatrix64F initialDoubleSupportICPpos,
   DenseMatrix64F initialDoubleSupportICPvel, DenseMatrix64F finalDoubleSupportICPpos, DenseMatrix64F finalDoubleSupportICPvel)
   {
      double doubleSupportTimePow2 = Math.pow(doubleSupportTime, 2);
      double doubleSupportTimePow3 = Math.pow(doubleSupportTime, 3);
      
      // Calculate time-dependency matrix for polynomial calculation (part of inversion problem)
      DenseMatrix64F TimeBoundaryConditionMatrix = new DenseMatrix64F(4, 4, true, -2.0, 1.0, 2.0, 1.0, 3.0 * doubleSupportTime, -2.0 * doubleSupportTime,
                                                      -3.0 * doubleSupportTime, -doubleSupportTime, 0.0, doubleSupportTimePow2, 0.0, 0.0,
                                                      -doubleSupportTimePow3, 0.0, 0.0, 0.0);

      // TimeBoundaryConditionMatrix.print();

      // Base = [                -2,         1,          2,              1; ...
      // 3*t2,       -2*t2,      -3*t2,     -t2; ...
      // 0,          t2_2,       0,         0; ...
      // -t2_3,      0,          0,       0];

      // Calculate DenominatorMatrix, so that T =  TimeBoundaryConditionMatrix*DenominatorMatrix, so that PolynomialParams = T*StateBoundaryConditionMatrix
      double denominator1 = -1.0 / doubleSupportTimePow3;
      double denominator2 = 1.0 / doubleSupportTimePow2;
      DenseMatrix64F DenominatorMatrix = CommonOps.diag(denominator1, denominator2, denominator1, denominator2);

      // System.out.println("DenominatorMatrix: ");
      // DenominatorMatrix.print();

      // Calculate PolynomialParams via inversion (see description above)
      DenseMatrix64F StateBoundaryConditionMatrix = new DenseMatrix64F(4, 3);
      EnhancedMatrixManipulator.setMatrixRowToVector(0, StateBoundaryConditionMatrix, initialDoubleSupportICPpos);
      EnhancedMatrixManipulator.setMatrixRowToVector(1, StateBoundaryConditionMatrix, initialDoubleSupportICPvel);
      EnhancedMatrixManipulator.setMatrixRowToVector(2, StateBoundaryConditionMatrix, finalDoubleSupportICPpos);
      EnhancedMatrixManipulator.setMatrixRowToVector(3, StateBoundaryConditionMatrix, finalDoubleSupportICPvel);

      // System.out.println("StateBoundaryConditionMatrix: ");
      // StateBoundaryConditionMatrix.print();

      DenseMatrix64F tempResultMatrix = new DenseMatrix64F(4, 3);
      DenseMatrix64F tempParamMatrix = new DenseMatrix64F(4, 3);
      CommonOps.mult(DenominatorMatrix, StateBoundaryConditionMatrix, tempResultMatrix);
      CommonOps.mult(TimeBoundaryConditionMatrix, tempResultMatrix, tempParamMatrix);
      CommonOps.transpose(tempParamMatrix);
      parameterMatrixToPack.set(tempParamMatrix);
   }

   public void calcDCMandECMPofTime(ArrayList<DenseMatrix64F> constantEquivalentCoPs, ArrayList<YoFramePoint> consideredFootStepLocationsFramePoints, double doubleSupportFirstStepFraction, double dcmConst,
                                    ArrayList<DenseMatrix64F> initialICPs, boolean isFirstStep, double initialTransferSupportTime, double doubleSupportTime,
                                    boolean isSingleSupport, double currentTime, double steppingTime)
   {
//    double currentTime = supportState.getCurrentTime();
//    double steppingTime = supportState.getSteppingTime();


      if (isSingleSupport)
      {
         double singleSupportComputationTime = currentTime + (1 - doubleSupportFirstStepFraction) * doubleSupportTime;
         JojosICPutilities.extrapolateDCMposAndVel(constantEquivalentCoPs.get(0), singleSupportComputationTime, dcmConst, initialICPs.get(0),
                 desiredDCMposOfTime, desiredDCMvelOfTime);
         desiredECMPofTime.set(constantEquivalentCoPs.get(0));
      }
      else
      {
         computeDoubleSupportPolynomialParams(constantEquivalentCoPs, consideredFootStepLocationsFramePoints, dcmConst, steppingTime, doubleSupportFirstStepFraction, initialICPs, isFirstStep,
                 initialTransferSupportTime, doubleSupportTime);

         double timePow3 = Math.pow(currentTime, 3.0);
         double timePow2 = Math.pow(currentTime, 2.0);
         DenseMatrix64F tempVector = new DenseMatrix64F(3, 1);

         DenseMatrix64F dcmPositionTimeVector = new DenseMatrix64F(4, 1, true, timePow3, timePow2, currentTime, 1);
         DenseMatrix64F dcmVelocityTimeVector = new DenseMatrix64F(4, 1, true, 3 * timePow2, 2 * currentTime, 1, 0);

         CommonOps.mult(doubleSupportParameterMatrix, dcmPositionTimeVector, desiredDCMposOfTime);
         CommonOps.mult(doubleSupportParameterMatrix, dcmVelocityTimeVector, desiredDCMvelOfTime);
         CommonOps.scale(-dcmConst, desiredDCMvelOfTime, tempVector);
         CommonOps.add(desiredDCMposOfTime, tempVector, desiredECMPofTime);
      }
   }

   public void updateSubFootListForSmoothICPTrajectory(ArrayList<DenseMatrix64F> constantEquivalentCoPs, ArrayList<YoFramePoint> footStepLocationsFramePoints,
           ArrayList<YoFramePoint> equivalentConstantCoPsFramePoints, ArrayList<YoFramePoint> consideredFootStepLocationsFramePoints,
           int numberOfConsideredFootstepLocations, ArrayList<DenseMatrix64F> equivalentConstantCoPsVectors, boolean isFirstStep)
   {
      int footListSize = footStepLocationsFramePoints.size();

      if (!(footListSize > 2))
      {
         finalCounter += 1;
      }

      if (finalCounter >= 2)
      {
         finalStepReached = true;
      }
      else
         finalStepReached = false;

      ArrayList<Integer> footListSelectionIndices = new ArrayList<Integer>();

      footListSelectionIndices.add(0);

      if ((footListSize <= 2) && finalStepReached)
      {
         tempFramePointI.set(footStepLocationsFramePoints.get(0).getFramePointCopy());

         tempFramePointIminus1.set(footStepLocationsFramePoints.get(1).getFramePointCopy());
         tempFramePointI.add(tempFramePointIminus1);
         tempFramePointI.scale(0.5);


         equivalentConstantCoPsFramePoints.get(0).set(tempFramePointI);

         consideredFootStepLocationsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));

      }
      else
      {
//         equivalentConstantCoPsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));
         
         if (isFirstStep)
         {
            tempFramePointI.set(footStepLocationsFramePoints.get(0).getFramePointCopy());  
            
            tempFramePointIminus1.set(footStepLocationsFramePoints.get(1).getFramePointCopy());
            tempFramePointI.add(tempFramePointIminus1);
            tempFramePointI.scale(0.5);
            
            
            equivalentConstantCoPsFramePoints.get(0).set(tempFramePointI);
         }
         else
         {
            equivalentConstantCoPsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));
         }

         consideredFootStepLocationsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));
      }

      boolean endWasReached = false;

      for (int i = 1; i < numberOfConsideredFootstepLocations; i++)
      {
         footListSelectionIndices.add(footListSelectionIndices.get(i - 1) + 1);

         if (endWasReached)
         {
            equivalentConstantCoPsFramePoints.get(i).set(equivalentConstantCoPsFramePoints.get(i - 1));

            consideredFootStepLocationsFramePoints.get(i).set(footStepLocationsFramePoints.get(footListSize - 1));
         }
         else if (i + 1 >= footListSize)
         {
            endWasReached = true;

            tempFramePointI.set(footStepLocationsFramePoints.get(i).getFramePointCopy());

            tempFramePointIminus1.set(footStepLocationsFramePoints.get(i - 1).getFramePointCopy());
            tempFramePointI.add(tempFramePointIminus1);
            tempFramePointI.scale(0.5);

            equivalentConstantCoPsFramePoints.get(i).set(tempFramePointI);

            consideredFootStepLocationsFramePoints.get(i).set(footStepLocationsFramePoints.get(footListSize - 1));
         }
         else
         {
            equivalentConstantCoPsFramePoints.get(i).set(footStepLocationsFramePoints.get(i));

            consideredFootStepLocationsFramePoints.get(i).set(footStepLocationsFramePoints.get(i));
         }
      }

      for (int i = 0; i < numberOfConsideredFootstepLocations; i++)
      {
         equivalentConstantCoPsVectors.get(i).set(0, 0, equivalentConstantCoPsFramePoints.get(i).getX());
         equivalentConstantCoPsVectors.get(i).set(1, 0, equivalentConstantCoPsFramePoints.get(i).getY());
         equivalentConstantCoPsVectors.get(i).set(2, 0, equivalentConstantCoPsFramePoints.get(i).getZ());
      }

      if (footListSize > 2)
      {
         footStepLocationsFramePoints.remove(0);
      }
   }
}
