package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.Matrix;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class SmoothCapturePointAdjustmentToolbox
{
   private static final int defaultSize = 100;

   private final DenseMatrix64F generalizedAlphaPrimeRowSegment = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeRowSegment = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedGammaPrimeMatrixSegment = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F generalizedAlphaBetaPrimeRowSegment = new DenseMatrix64F(1, defaultSize);

   private final DenseMatrix64F alphaPrimeRowSegment = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F betaPrimeRowSegment = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F gammaPrimeMatrixSegment2 = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F alphaBetaPrimeRowSegment2 = new DenseMatrix64F(1, defaultSize);

   private final DenseMatrix64F boundaryConditionMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F boundaryConditionMatrixInverse = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F boundaryConditionVector = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F polynomialCoefficientCombinedVectorAdjustment = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F polynomialCoefficientVectorAdjustmentSegment1 = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F polynomialCoefficientVectorAdjustmentSegment2 = new DenseMatrix64F(defaultSize, 1);

   private final LinearSolver<DenseMatrix64F> pseudoInverseSolver = new SolvePseudoInverseSvd();

   private List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList = new ArrayList<FrameTuple3D<?, ?>>();

   private final SmoothCapturePointToolbox icpToolbox;

   public SmoothCapturePointAdjustmentToolbox(SmoothCapturePointToolbox smoothCapturePointToolbox)
   {
      this.icpToolbox = smoothCapturePointToolbox;

      icpQuantityInitialConditionList.add(new FramePoint3D());
      while (icpQuantityInitialConditionList.size() < defaultSize)
      {
         icpQuantityInitialConditionList.add(new FrameVector3D());
      }
   }

   public void setICPInitialConditions(double localTime, List<FramePoint3D> exitCornerPointsFromCoPs, List<FrameTrajectory3D> copPolynomials3D,
                                       int currentSwingSegment, double omega0)
   {
      if (currentSwingSegment < 0)
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(0);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            copPolynomial3D.getDerivative(i, localTime, icpQuantityInitialCondition);
         }
      }
      else
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(currentSwingSegment);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, localTime, i, copPolynomial3D,
                                                                            exitCornerPointsFromCoPs.get(currentSwingSegment), icpQuantityInitialCondition);
         }
      }
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing(List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack,
                                                            List<FrameTrajectory3D> copPolynomials3D, double omega0)
   {
      adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, copPolynomials3D, icpQuantityInitialConditionList, entryCornerPointsToPack,
                                                     exitCornerPointsToPack);
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing3D(double omega0, List<FrameTrajectory3D> copPolynomials3D,
                                                              List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList,
                                                              List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack)
   {
      FrameTrajectory3D cmpPolynomial3DSegment1 = copPolynomials3D.get(0);
      FrameTrajectory3D cmpPolynomial3DSegment2 = copPolynomials3D.get(1);
      FramePoint3D icpPositionFinalSegment2 = exitCornerPointsToPack.get(1);
      for (Direction direction : Direction.values)
      {
         Trajectory cmpPolynomialSegment1 = cmpPolynomial3DSegment1.getTrajectory(direction);
         Trajectory cmpPolynomialSegment2 = cmpPolynomial3DSegment2.getTrajectory(direction);

         double icpPositionFinalSegment2Scalar = icpPositionFinalSegment2.getElement(direction.getIndex());

         int numberOfCoefficients = cmpPolynomialSegment1.getNumberOfCoefficients();
         int numberOfConstrainedDerivatives = numberOfCoefficients / 2;

         initializeMatrices1D(numberOfCoefficients, numberOfConstrainedDerivatives);

         populateBoundaryConditionMatrices1D(omega0, direction, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1,
                                             cmpPolynomialSegment2, icpQuantityInitialConditionList, icpPositionFinalSegment2Scalar);
         computeAdjustedPolynomialCoefficientVectors1D(numberOfCoefficients);
         adjustCMPPolynomials(cmpPolynomialSegment1, cmpPolynomialSegment2);
      }
      icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsToPack, exitCornerPointsToPack, copPolynomials3D, omega0);
   }

   private void adjustCMPPolynomials(Trajectory cmpPolynomialSegment1, Trajectory cmpPolynomialSegment2)
   {
      cmpPolynomialSegment1.setDirectly(polynomialCoefficientVectorAdjustmentSegment1);
      cmpPolynomialSegment2.setDirectly(polynomialCoefficientVectorAdjustmentSegment2);
   }

   private void populateBoundaryConditionMatrices1D(double omega0, Direction direction, int numberOfCoefficients, int numberOfConstrainedDerivatives,
                                                    Trajectory cmpPolynomialSegment1, Trajectory cmpPolynomialSegment2,
                                                    List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList, double icpPositionFinalSegment2Scalar)
   {
      calculateGeneralizedICPMatricesOnCMPSegment2(omega0, cmpPolynomialSegment2);

      // TODO: check whether division always integer
      for (int i = 0; i < numberOfConstrainedDerivatives; i++)
      {
         double icpQuantityInitialConditionScalar = icpQuantityInitialConditionList.get(i).getElement(direction.getIndex());
         calculateGeneralizedICPMatricesOnCMPSegment1(omega0, i, cmpPolynomialSegment1);
         setGeneralizedBoundaryConstraints(i, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1, cmpPolynomialSegment2,
                                           icpQuantityInitialConditionScalar, icpPositionFinalSegment2Scalar);
      }
   }

   private void setGeneralizedBoundaryConstraintICP0(int order, int numberOfCoefficients,
                                                     double icpQuantityInitialConditionScalar, double icpPositionFinalSegment2,
                                                     DenseMatrix64F generalizedAlphaBetaPrimeRowSegment1, DenseMatrix64F generalizedGammaPrimeMatrixSegment1,
                                                     DenseMatrix64F alphaBetaPrimeRowSegment2, DenseMatrix64F gammaPrimeMatrixSegment2)
   {
      double generalizedBoundaryConditionValue = icpQuantityInitialConditionScalar
            - generalizedGammaPrimeMatrixSegment1.get(0, 0) * gammaPrimeMatrixSegment2.get(0, 0) * icpPositionFinalSegment2;
      boundaryConditionVector.set(order, 0, generalizedBoundaryConditionValue);

      MatrixTools.setMatrixBlock(boundaryConditionMatrix, order, 0, generalizedAlphaBetaPrimeRowSegment1, 0, 0, generalizedAlphaBetaPrimeRowSegment1.numRows,
                                 generalizedAlphaBetaPrimeRowSegment1.numCols, 1.0);

      MatrixTools.setMatrixBlock(boundaryConditionMatrix, order, numberOfCoefficients, alphaBetaPrimeRowSegment2, 0, 0, alphaBetaPrimeRowSegment2.numRows,
                                 alphaBetaPrimeRowSegment2.numCols, generalizedGammaPrimeMatrixSegment1.get(0, 0));
   }

   private void setGeneralizedBoundaryConstraintCMP0(int order, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1)
   {
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();

      boundaryConditionVector.set(order + numberOfConstrainedDerivatives, 0, cmpPolynomialSegment1.getDerivative(order, tInitial1));

      DenseMatrix64F xPowersDerivativeVector = cmpPolynomialSegment1.getXPowersDerivativeVector(order, tInitial1);
      MatrixTools.setMatrixBlock(boundaryConditionMatrix, order + numberOfConstrainedDerivatives, 0, xPowersDerivativeVector, 0, 0,
                                 xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, 1.0);
   }

   private void setGeneralizedBoundaryConstraintCMP1(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1,
                                                     Trajectory cmpPolynomialSegment2)
   {
      double tFinal1 = cmpPolynomialSegment1.getFinalTime();
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();

      boundaryConditionVector.set(order + 2 * numberOfConstrainedDerivatives, 0, 0.0);

      DenseMatrix64F xPowersDerivativeVector = cmpPolynomialSegment1.getXPowersDerivativeVector(order, tFinal1);
      MatrixTools.setMatrixBlock(boundaryConditionMatrix, order + 2 * numberOfConstrainedDerivatives, 0, xPowersDerivativeVector, 0, 0,
                                 xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, -1.0);

      xPowersDerivativeVector = cmpPolynomialSegment2.getXPowersDerivativeVector(order, tInitial2);
      MatrixTools.setMatrixBlock(boundaryConditionMatrix, order + 2 * numberOfConstrainedDerivatives, numberOfCoefficients, xPowersDerivativeVector, 0, 0,
                                 xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, 1.0);
   }

   private void setGeneralizedBoundaryConstraintCMP2(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment2)
   {
      double tFinal2 = cmpPolynomialSegment2.getFinalTime();

      boundaryConditionVector.set(order + 3 * numberOfConstrainedDerivatives, 0, cmpPolynomialSegment2.getDerivative(order, tFinal2));

      DenseMatrix64F xPowersDerivativeVector = cmpPolynomialSegment2.getXPowersDerivativeVector(order, tFinal2);
      MatrixTools.setMatrixBlock(boundaryConditionMatrix, order + 3 * numberOfConstrainedDerivatives, numberOfCoefficients, xPowersDerivativeVector,
                                 0, 0, xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, 1.0);
   }

   private void computeAdjustedPolynomialCoefficientVectors1D(int numberOfCoefficients)
   {
      // Uses the Moore-Penrose pseudo-inverse to counter bad conditioning of boundaryConditionMatrix
      pseudoInverseSolver.setA(boundaryConditionMatrix);
      pseudoInverseSolver.solve(boundaryConditionVector, polynomialCoefficientCombinedVectorAdjustment);

      // FIXME generating garbage
      polynomialCoefficientVectorAdjustmentSegment1.set(CommonOps.extract(polynomialCoefficientCombinedVectorAdjustment, 0, numberOfCoefficients, 0, 1));
      polynomialCoefficientVectorAdjustmentSegment2.set(CommonOps.extract(polynomialCoefficientCombinedVectorAdjustment, numberOfCoefficients, 2 * numberOfCoefficients, 0, 1));
   }

   private void calculateGeneralizedICPMatricesOnCMPSegment2(double omega0, Trajectory cmpPolynomialSegment2)
   {
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();
      icpToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial2, 0, cmpPolynomialSegment2, alphaPrimeRowSegment, betaPrimeRowSegment,
                                                                 gammaPrimeMatrixSegment2, alphaBetaPrimeRowSegment2);
   }

   private void calculateGeneralizedICPMatricesOnCMPSegment1(double omega0, int derivativeOrder, Trajectory cmpPolynomialSegment1)
   {
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();
      icpToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial1, derivativeOrder, cmpPolynomialSegment1, generalizedAlphaPrimeRowSegment,
                                                                 generalizedBetaPrimeRowSegment, generalizedGammaPrimeMatrixSegment,
                                                                 generalizedAlphaBetaPrimeRowSegment);
   }

   private void setGeneralizedBoundaryConstraints(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1,
                                                  Trajectory cmpPolynomialSegment2, double icpQuantityInitialConditionScalar,
                                                  double icpPositionFinalSegment2Scalar)
   {
      setGeneralizedBoundaryConstraintICP0(order, numberOfCoefficients, icpQuantityInitialConditionScalar, icpPositionFinalSegment2Scalar,
                                           generalizedAlphaBetaPrimeRowSegment, generalizedGammaPrimeMatrixSegment,
                                           alphaBetaPrimeRowSegment2, gammaPrimeMatrixSegment2);
      setGeneralizedBoundaryConstraintCMP0(order, numberOfConstrainedDerivatives, cmpPolynomialSegment1);
      setGeneralizedBoundaryConstraintCMP1(order, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1, cmpPolynomialSegment2);
      setGeneralizedBoundaryConstraintCMP2(order, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment2);
   }

   private void initializeMatrices1D(int numberOfCoefficients, int numberOfConstrainedDerivatives)
   {
      boundaryConditionMatrix.reshape(4 * numberOfConstrainedDerivatives, 2 * numberOfCoefficients);
      boundaryConditionMatrixInverse.reshape(2 * numberOfCoefficients, 4 * numberOfConstrainedDerivatives);
      boundaryConditionVector.reshape(4 * numberOfConstrainedDerivatives, 1);
      polynomialCoefficientCombinedVectorAdjustment.reshape(2 * numberOfCoefficients, 1);

      polynomialCoefficientVectorAdjustmentSegment1.reshape(numberOfCoefficients, 1);
      polynomialCoefficientVectorAdjustmentSegment2.reshape(numberOfCoefficients, 1);

      generalizedAlphaPrimeRowSegment.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeRowSegment.reshape(1, numberOfCoefficients);
      generalizedGammaPrimeMatrixSegment.reshape(1, 1);
      generalizedAlphaBetaPrimeRowSegment.reshape(1, numberOfCoefficients);

      alphaPrimeRowSegment.reshape(1, numberOfCoefficients);
      betaPrimeRowSegment.reshape(1, numberOfCoefficients);
      gammaPrimeMatrixSegment2.reshape(1, 1);
      alphaBetaPrimeRowSegment2.reshape(1, numberOfCoefficients);
   }
}
