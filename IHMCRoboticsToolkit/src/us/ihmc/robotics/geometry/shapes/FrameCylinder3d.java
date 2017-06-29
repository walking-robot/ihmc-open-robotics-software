package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameCylinder3d extends FrameShape3d<FrameCylinder3d, Cylinder3D>
{
   private Cylinder3D cylinder3d;

   public FrameCylinder3d(FrameCylinder3d other)
   {
      this(other.referenceFrame, other.cylinder3d);
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Cylinder3D());
      cylinder3d = getGeometryObject();
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame, Cylinder3D cylinder3d)
   {
      super(referenceFrame, new Cylinder3D(cylinder3d));
      this.cylinder3d = getGeometryObject();
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame, double height, double radius)
   {
      super(referenceFrame, new Cylinder3D(height, radius));
      cylinder3d = getGeometryObject();
   }
   
   public FrameCylinder3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double height, double radius)
   {
      super(referenceFrame, new Cylinder3D(configuration, height, radius));
      cylinder3d = getGeometryObject();
   }

   public Cylinder3D getCylinder3d()
   {
      return cylinder3d;
   }

   public double getRadius()
   {
      return cylinder3d.getRadius();
   }

   public double getHeight()
   {
      return cylinder3d.getHeight();
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrame: " + referenceFrame + ")\n");
      builder.append(cylinder3d.toString());

      return builder.toString();
   }
}
