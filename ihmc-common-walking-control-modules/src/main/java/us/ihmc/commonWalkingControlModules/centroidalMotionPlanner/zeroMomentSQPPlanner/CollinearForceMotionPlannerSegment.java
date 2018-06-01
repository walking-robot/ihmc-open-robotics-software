package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;

public class CollinearForceMotionPlannerSegment
{
   private double duration;
   private ContactState segmentContactState;

   public void setContactState(ContactState contactStateToSave)
   {
      this.segmentContactState = contactStateToSave;
   }

   public ContactState getContactState()
   {
      return segmentContactState;
   }

   public void setSegmentDuration(double segmentDuration)
   {
      this.duration = segmentDuration;
   }

   public double getSegmentDuration()
   {
      return duration;
   }

   public String toString()
   {
      return "Duration: " + duration + ", SupportState: " + segmentContactState.toString();
   }
}