package us.ihmc.avatar.joystickBasedJavaFXController;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.TabPane;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class GraspingObjectPaneController
{
   private JavaFXMessager messager;

   @FXML
   private Slider sliderSphereRadius;

   @FXML
   private Slider sliderCylinderRadius;

   @FXML
   private Slider sliderCylinderHeight;

   @FXML
   private Slider sliderTorusRadius;

   @FXML
   private Slider sliderTorusTube;

   @FXML
   private Slider sliderBoxLength;

   @FXML
   private Slider sliderBoxWidth;

   @FXML
   private Slider sliderBoxHeight;

   @FXML
   private TabPane tapPaneShape;

   public GraspingObjectPaneController()
   {

   }

   public void initialize(JavaFXMessager messager)
   {
      this.messager = messager;

      sliderSphereRadius.setValue(0.1);
      messager.bindBidirectional(GraspingJavaFXTopics.SphereRadius, sliderSphereRadius.valueProperty(), createConverter(), true);
      
      messager.bindBidirectional(GraspingJavaFXTopics.CylinderRadius, sliderCylinderRadius.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.CylinderHeight, sliderCylinderHeight.valueProperty(), createConverter(), true);
      
      messager.bindBidirectional(GraspingJavaFXTopics.TorusRadius, sliderTorusRadius.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.TorusTubeRadius, sliderTorusTube.valueProperty(), createConverter(), true);
      
//      messager.bindBidirectional(GraspingJavaFXTopics., sliderBoxLength.valueProperty(), createConverter(), true);
//      messager.bindBidirectional(GraspingJavaFXTopics.SphereRadius, sliderBoxWidth.valueProperty(), createConverter(), true);
//      messager.bindBidirectional(GraspingJavaFXTopics.SphereRadius, sliderBoxHeight.valueProperty(), createConverter(), true);
      
      
      int selectedIndex = tapPaneShape.getSelectionModel().getSelectedIndex();
   }

   private PropertyToMessageTypeConverter<Double, Number> createConverter()
   {
      return new PropertyToMessageTypeConverter<Double, Number>()
      {
         @Override
         public Double convert(Number propertyValue)
         {
            return propertyValue.doubleValue();
         }

         @Override
         public Number interpret(Double messageContent)
         {
            return messageContent;
         }
      };
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
}
