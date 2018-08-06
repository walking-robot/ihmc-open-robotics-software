package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.euclid.geometry.Sphere3D;
import us.ihmc.euclid.geometry.Torus3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;

/**
 * What to do this.
 * Handler grasping object.
 * Communicate with WholeBodyTrajectoryToolbox.
 * Control fingers.
 * Visualize object and preview of motion.
 */

public class GraspingJavaFXController
{
   private final JavaFXMessager messager;

   // //TODO : object color selector.

   // //TODO : get button from xbox to change controlled object.

   // //TODO : pos, orientation controlled by xbox.

   // //TODO : xbox can create object.

   // TODO : xbox can send to toolbox. 

   // TODO : xbox can accept motion.   ghost robot.

   // TODO : show up pdf or png to show instruction for xbox.

   private final AtomicReference<List<Node>> objectsToVisualizeReference = new AtomicReference<>(new ArrayList<>());

   private final AtomicReference<Double> sphereRadius;

   private final AtomicReference<Double> cylinderRadius;
   private final AtomicReference<Double> cylinderHeight;

   private final AtomicReference<Double> torusRadius;
   private final AtomicReference<Double> torusTube;

   private final AnimationTimer animationTimer;

   private final Group rootNode = new Group();

   private final RigidBodyTransform defaultTransformToCreate = new RigidBodyTransform();
   private final Point3D controlPosition = new Point3D();
   private final RotationMatrix controlOrientation = new RotationMatrix();

   private final static double ratioJoyStickToPosition = 0.01;
   private final static double ratioJoyStickToRotation = 0.02;

   private final static double lengthOfControlFrame = 0.2;

   private final DoubleProperty velocityXProperty = new SimpleDoubleProperty(this, "velocityXProperty", 0.0);
   private final DoubleProperty velocityYProperty = new SimpleDoubleProperty(this, "velocityYProperty", 0.0);
   private final DoubleProperty velocityZProperty = new SimpleDoubleProperty(this, "velocityZProperty", 0.0);
   private final DoubleProperty velocityRollProperty = new SimpleDoubleProperty(this, "velocityRollProperty", 0.0);
   private final DoubleProperty velocityPitchProperty = new SimpleDoubleProperty(this, "velocityPitchProperty", 0.0);
   private final DoubleProperty velocityYawProperty = new SimpleDoubleProperty(this, "velocityYawProperty", 0.0);

   private ShapeToCreate shapeToCreate = null;
   private int indexOfSelectedObject = 0;
   private final List<Shape3D<?>> listOfShape3D = new ArrayList<>();

   private enum ShapeToCreate
   {
      Sphere, Cylinder, Torus, Box;
      public ShapeToCreate nextShape()
      {
         int nextOrdinal = this.ordinal() + 1;
         for (ShapeToCreate shape : ShapeToCreate.values())
            if (shape.ordinal() == nextOrdinal)
               return shape;
         return Sphere;
      }
   }

   public GraspingJavaFXController(JavaFXMessager messager)
   {
      this.messager = messager;

      sphereRadius = messager.createInput(GraspingJavaFXTopics.SphereRadius, 0.1);

      cylinderRadius = messager.createInput(GraspingJavaFXTopics.CylinderRadius, 0.1);
      cylinderHeight = messager.createInput(GraspingJavaFXTopics.CylinderHeight, 0.1);

      torusRadius = messager.createInput(GraspingJavaFXTopics.TorusRadius, 0.1);
      torusTube = messager.createInput(GraspingJavaFXTopics.TorusTubeRadius, 0.1);

      // temp adding Shape3D
      defaultTransformToCreate.appendTranslation(1.0, 0.0, 0.0);

      // register xbox events
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonSelectState, state -> clearObjects(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonStartState, state -> switchShapeToCreate(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonAState, state -> createObject(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonBState, state -> switchSelectedObject(state));

      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftStickYAxis, this::appendingXAxis);
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftStickXAxis, this::appendingYAxis);
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonLeftBumperState, state -> appendingZAxisPositive(state));
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftTriggerAxis, this::appendingZAxisNegative);

      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightStickYAxis, this::appendingPitch);
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightStickXAxis, this::appendingRoll);
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonRightBumperState, state -> appendingYawPositive(state));
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightTriggerAxis, this::appendingYawNegative);

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long arg0)
         {
            updateSelectedObject();
            updateVisualizationObjects();
         }
      };
   }

   private void updateSelectedObject()
   {
      controlPosition.add(velocityXProperty.getValue(), velocityYProperty.getValue(), velocityZProperty.getValue());
      controlOrientation.prependRollRotation(velocityRollProperty.getValue());
      controlOrientation.prependPitchRotation(velocityPitchProperty.getValue());
      controlOrientation.prependYawRotation(velocityYawProperty.getValue());
      // if append rotations, it rotates with current reference.
   }

   private void appendingRoll(double alpha)
   {
      velocityRollProperty.set(-alpha * ratioJoyStickToRotation);
   }

   private void appendingPitch(double alpha)
   {
      velocityPitchProperty.set(alpha * ratioJoyStickToRotation);
   }

   private void appendingYawNegative(double alpha)
   {
      velocityYawProperty.set(alpha * ratioJoyStickToRotation);
   }

   private void appendingYawPositive(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
         velocityYawProperty.set(ratioJoyStickToRotation);
      else
         velocityYawProperty.set(0.0);
   }

   private void appendingXAxis(double alpha)
   {
      velocityXProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingYAxis(double alpha)
   {
      velocityYProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingZAxisNegative(double alpha)
   {
      velocityZProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingZAxisPositive(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
         velocityZProperty.set(ratioJoyStickToPosition);
      else
         velocityZProperty.set(0.0);
   }

   private void switchShapeToCreate(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         if (shapeToCreate == null)
            shapeToCreate = ShapeToCreate.Sphere;
         else
            shapeToCreate = shapeToCreate.nextShape();

         System.out.println("switch shape to " + shapeToCreate);
      }
   }

   private void createObject(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         if (shapeToCreate != null)
         {
            System.out.println("create " + shapeToCreate);
            switch (shapeToCreate)
            {
            case Sphere:
               Sphere3D sphere = new Sphere3D(sphereRadius.get().doubleValue());
               sphere.appendTranslation(defaultTransformToCreate.getTranslationVector());
               listOfShape3D.add(sphere);
               break;
            case Cylinder:
               Cylinder3D cylinder = new Cylinder3D(cylinderHeight.get().doubleValue(), cylinderRadius.get().doubleValue());
               cylinder.setPose(defaultTransformToCreate);
               listOfShape3D.add(cylinder);
               break;
            case Torus:
               Torus3D torus = new Torus3D(torusRadius.get().doubleValue(), torusTube.get().doubleValue());
               torus.setPose(defaultTransformToCreate);
               listOfShape3D.add(torus);
               break;
            case Box:
               // TODO : implements with UI.
               break;
            default:
               break;
            }

            indexOfSelectedObject = listOfShape3D.size() - 1;
            shapeToCreate = null;
         }
         else
            System.out.println("shape should be selected.");
      }
   }

   private void switchSelectedObject(ButtonState state)
   {
      int numberOfObjects = listOfShape3D.size();

      if (numberOfObjects < 1)
         return;

      if (state == ButtonState.RELEASED)
         return;

      indexOfSelectedObject++;
      if (indexOfSelectedObject == numberOfObjects)
         indexOfSelectedObject = 0;

      listOfShape3D.get(indexOfSelectedObject).getPosition(controlPosition);
      listOfShape3D.get(indexOfSelectedObject).getOrientation(controlOrientation);
   }

   private void clearObjects(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         listOfShape3D.clear();
         indexOfSelectedObject = 0;
      }
   }

   private void controlObject(Shape3D<?> shape3D)
   {
      shape3D.setPosition(controlPosition);
      shape3D.setOrientation(controlOrientation);

      if (shape3D instanceof Sphere3D)
      {
         ((Sphere3D) shape3D).setRadius(sphereRadius.get().doubleValue());
      }
      else if (shape3D instanceof Cylinder3D)
      {
         ((Cylinder3D) shape3D).setRadius(cylinderRadius.get().doubleValue());
         ((Cylinder3D) shape3D).setHeight(cylinderHeight.get().doubleValue());
      }
      else if (shape3D instanceof Torus3D)
      {
         ((Torus3D) shape3D).setRadii(torusRadius.get().doubleValue(), torusTube.get().doubleValue());
      }
      else if (shape3D instanceof Box3D)
      {
         ;
      }
   }

   private Node createVisualizationObject(Shape3D<?> shape3D, boolean selected)
   {
      Point3D position = new Point3D(shape3D.getPosition());
      Quaternion orientation = new Quaternion(shape3D.getOrientation());

      MeshDataBuilder meshBuilder = new MeshDataBuilder();
      if (shape3D instanceof Sphere3D)
      {
         meshBuilder.addSphere(((Sphere3D) shape3D).getRadius(), new Point3D());
      }
      else if (shape3D instanceof Cylinder3D)
      {
         meshBuilder.addCylinder(((Cylinder3D) shape3D).getHeight(), ((Cylinder3D) shape3D).getRadius(), new Point3D());
      }
      else if (shape3D instanceof Box3D)
      {
         meshBuilder.addBox((float) ((Box3D) shape3D).getLength(), (float) ((Box3D) shape3D).getWidth(), (float) ((Box3D) shape3D).getHeight());
      }
      else if (shape3D instanceof Torus3D)
      {
         meshBuilder.addMesh(MeshDataGenerator.ArcTorus(0.0, 2 * Math.PI, ((Torus3D) shape3D).getRadius(), ((Torus3D) shape3D).getTubeRadius(), 10));
      }

      MeshDataHolder meshDataHolder = meshBuilder.generateMeshDataHolder();

      meshDataHolder = MeshDataHolder.rotate(meshDataHolder, orientation);
      meshDataHolder = MeshDataHolder.translate(meshDataHolder, position);

      Mesh mesh = JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder, true);

      MeshView meshView = new MeshView(mesh);
      if (selected)
         meshView.setMaterial(new PhongMaterial(Color.RED));
      else
         meshView.setMaterial(new PhongMaterial(Color.BEIGE));
      return meshView;
   }

   private void updateVisualizationObjects()
   {
      List<Node> objectsToPutReference = new ArrayList<Node>();
      for (int i = 0; i < listOfShape3D.size(); i++)
      {
         Shape3D<?> objectToVisualize = listOfShape3D.get(i);
         boolean selected = false;
         if (i == indexOfSelectedObject)
         {
            controlObject(objectToVisualize);
            selected = true;
         }

         Node node = createVisualizationObject(listOfShape3D.get(i), selected);
         objectsToPutReference.add(node);
      }

      JavaFXCoordinateSystem controlCoordinateSystem = new JavaFXCoordinateSystem(lengthOfControlFrame);
      Affine controlTransform = JavaFXTools.createAffineFromQuaternionAndTuple(new Quaternion(controlOrientation), controlPosition);
      controlCoordinateSystem.getTransforms().add(controlTransform);
      objectsToPutReference.add(controlCoordinateSystem);

      objectsToVisualizeReference.set(objectsToPutReference);

      List<Node> objectsToVisualize = objectsToVisualizeReference.getAndSet(null);
      ObservableList<Node> children = rootNode.getChildren();

      if (objectsToVisualize != null)
      {
         children.clear();
         children.addAll(objectsToVisualize);
      }
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
