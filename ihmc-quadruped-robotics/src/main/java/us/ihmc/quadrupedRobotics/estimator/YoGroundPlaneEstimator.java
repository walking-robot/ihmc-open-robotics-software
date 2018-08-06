package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class YoGroundPlaneEstimator extends GroundPlaneEstimator
{
   private final YoFramePoint3D yoGroundPlanePoint;
   private final YoFrameVector3D yoGroundPlaneNormal;
   private final YoFrameYawPitchRoll yoGroundPlaneOrientation;

   public YoGroundPlaneEstimator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this("", parentRegistry, graphicsListRegistry, YoAppearance.Glass());
   }

   public YoGroundPlaneEstimator(String prefix, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, AppearanceDefinition groundPlaneAppearance)
   {
      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      yoGroundPlanePoint = new YoFramePoint3D(prefix + "GroundPlanePoint", ReferenceFrame.getWorldFrame(), registry);
      yoGroundPlaneNormal = new YoFrameVector3D(prefix + "GroundPlaneNormal", ReferenceFrame.getWorldFrame(), registry);
      yoGroundPlaneOrientation = new YoFrameYawPitchRoll(prefix + "GroundPlaneOrientation", ReferenceFrame.getWorldFrame(), registry);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (graphicsListRegistry != null)
      {
         Graphics3DObject groundPlaneGraphic = new Graphics3DObject();
         groundPlaneGraphic.addCylinder(0.005, 0.5, groundPlaneAppearance);
         YoGraphicShape yoGroundPlaneGraphic = new YoGraphicShape(prefix + "GroundPlaneEstimate", groundPlaneGraphic, yoGroundPlanePoint, yoGroundPlaneOrientation, 1.0);
         graphicsListRegistry.registerYoGraphic(prefix + "GroundPlaneEstimate", yoGroundPlaneGraphic);
      }
   }

   @Override
   public void compute()
   {
      super.compute();

      yoGroundPlaneNormal.set(groundPlaneFrameNormal);

      yoGroundPlanePoint.set(groundPlaneFramePoint);
      yoGroundPlaneOrientation.setYawPitchRoll(0.0, getPitch(), getRoll());
   }
}
