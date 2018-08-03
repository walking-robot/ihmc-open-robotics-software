package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController.ButtonXState;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.euclid.geometry.Sphere3D;
import us.ihmc.euclid.geometry.Torus3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

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

   // TODO : object color selector.

   // TODO : get button from xbox to change controlled object.

   // TODO : pos, orientation controlled by xbox.

   // TODO : xbox can create object.

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

   private int indexOfSelectedObject = 0;
   private final List<Shape3D<?>> listOfShape3D = new ArrayList<>();

   private enum ShapeToCreate
   {
      Sphere, Cylinder, Torus, Box
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
      listOfShape3D.add(new Sphere3D(new Point3D(1, 0, 1), 0.1));
      listOfShape3D.add(new Sphere3D(new Point3D(2, 0, 1), 0.1));
      listOfShape3D.add(new Sphere3D(new Point3D(3, 0, 1), 0.1));
      RigidBodyTransform tempTransform = new RigidBodyTransform();
      tempTransform.appendTranslation(0.5, 0, 0.0);
      tempTransform.appendPitchRotation(0.1);
      listOfShape3D.add(new Cylinder3D(tempTransform, 0.3, 0.05));
      
      // register xbox events
      messager.registerTopicListener(ButtonXState, state -> switchSelectedObject(state));
      
      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long arg0)
         {
            //objectVisualizerUpdate();

            updateVisualizationObjects();
         }
      };
   }
   
   private void switchSelectedObject(ButtonState state)
   {
      int numberOfObjects = listOfShape3D.size();
      
      if(numberOfObjects < 1)
         return;
      
      if(state == ButtonState.RELEASED)
         return;
      
      indexOfSelectedObject++;
      if(indexOfSelectedObject == numberOfObjects)
         indexOfSelectedObject = 0;
   }
   
   private void controlObject()
   {
      
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
         ;
      }

      MeshDataHolder meshDataHolder = meshBuilder.generateMeshDataHolder();
      meshDataHolder = MeshDataHolder.translate(meshDataHolder, position);
      meshDataHolder = MeshDataHolder.rotate(meshDataHolder, orientation);

      Mesh mesh = JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder, true);

      MeshView meshView = new MeshView(mesh);
      if(selected)
         meshView.setMaterial(new PhongMaterial(Color.RED));
      else
         meshView.setMaterial(new PhongMaterial(Color.BEIGE));
      return meshView;
   }

   private void updateVisualizationObjects()
   {
      List<Node> objectsToPutReference = new ArrayList<Node>();
      for(int i=0;i<listOfShape3D.size();i++)
      {
         boolean selected = false;
         if(i == indexOfSelectedObject)
            selected= true;
         Node node = createVisualizationObject(listOfShape3D.get(i), selected);
         objectsToPutReference.add(node);
      }
      objectsToVisualizeReference.set(objectsToPutReference);

      List<Node> objectsToVisualize = objectsToVisualizeReference.getAndSet(null);
      ObservableList<Node> children = rootNode.getChildren();

      if (objectsToVisualize != null)
      {
         children.clear();
         children.addAll(objectsToVisualize);
      }
   }

   private Node createSphere(Point3DReadOnly position, Color footColor)
   {
      MeshDataBuilder meshBuilder = new MeshDataBuilder();
      meshBuilder.addSphere(sphereRadius.get().doubleValue(), position);
      MeshDataHolder meshDataHolder = meshBuilder.generateMeshDataHolder();
      Mesh mesh = JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder, true);
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(footColor));
      return meshView;
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
