#include "OpenDoor.h"


#include "CallGUI.h"
#include "LogicalNavigation.h"

#include "ActionFactory.h"

#include "actasp/AspFluent.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_kr_execution/AspRule.h"
#include "bwi_kr_execution/AspFluent.h"

#include <ros/ros.h>

/*******************************************************
*                   Added this                 *
********************************************************/
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

/*******************************************************
*                   segbot_led Headers                 *
********************************************************/
#include "bwi_msgs/LEDActionResult.h"
#include "bwi_msgs/LEDAnimations.h"
#include "bwi_msgs/LEDClear.h"
#include "bwi_msgs/LEDControlAction.h"

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_services/SpeakMessage.h"


using namespace std;

namespace bwi_krexec {

OpenDoor::OpenDoor() : 
            door(),
            done(false),
            asked(false),
            open(false),
            failed(false),
            startTime(){}




void OpenDoor::run() {
	
	/* stuff added */
  ros::NodeHandle n;

  ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");
  bwi_services::SpeakMessage speak_srv;

  actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
  ac.waitForServer();
  bwi_msgs::LEDControlGoal goal;
	//end edit
	
  if(!asked) {
	//added  
	goal.type.led_animations = bwi_msgs::LEDAnimations::NEED_ASSIST;
    goal.timeout = ros::Duration(0);
    ac.sendGoal(goal);

    speak_srv.request.message = "Can you open door " + door + ", please?";
    speak_message_client.call(speak_srv);
    //end edit
    
    CallGUI askToOpen("askToOpen", CallGUI::DISPLAY,  "Can you open door " + door + ", please?");
    askToOpen.run();
    asked = true;
    startTime = ros::Time::now();
  }
  
  if(!open) {
    vector<string> params;
    params.push_back(door);
    LogicalNavigation senseDoor("sensedoor",params);
   
    senseDoor.run();
    
    //check if door is open
    //ros::NodeHandle n;
    ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
    
    bwi_kr_execution::AspFluent openFluent;
    openFluent.name = "open";
    openFluent.timeStep = 0;
    openFluent.variables.push_back(door);
    
    bwi_kr_execution::AspRule rule;
    rule.head.push_back(openFluent);
    
    bwi_kr_execution::CurrentStateQuery csq;
    csq.request.query.push_back(rule);
    
    currentClient.call(csq);
    
    open = csq.response.answer.satisfied;
    
    if(!open && (ros::Time::now() - startTime) > ros::Duration(15.0)) {
      failed = true;
      done = true;
    }
    
    ROS_DEBUG_STREAM( "door open: " << open );
  }
  
  if(open) {
	
	//edit
	ac.cancelAllGoals();

    speak_srv.request.message = "Thanks!";
    speak_message_client.call(speak_srv);
    //end edit  
	  
    CallGUI askToOpen("thank", CallGUI::DISPLAY,  "Thanks!");
    askToOpen.run();
    done = true;
  }

}  
  
actasp::Action* OpenDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  OpenDoor *newAction = new OpenDoor();
  newAction->door = fluent.getParameters().at(0);
  
  return newAction;
}

std::vector<std::string> OpenDoor::getParameters() const {
  vector<string> param;
  param.push_back(door);
  return param;
}


ActionFactory openDoorFactory(new OpenDoor(), false);
  
}
