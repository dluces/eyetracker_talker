/**
* @file /eyetracker_talker/src/lib/eyetracker_talker.cpp
*
* @brief File comment
**/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/eyetracker_talker/eyetracker_talker.hpp"

#include <sstream>
#include <iostream>

namespace eyetracker_talker
{
  // ID of the global interactor that provides our data stream; must be unique within the application.
  const TX_STRING EyeTracker::InteractorId = "EyeTrackerTalker";

  // global variables
  TX_HANDLE EyeTracker::g_hGlobalInteractorSnapshot = TX_EMPTY_HANDLE;
  TX_CONTEXTHANDLE EyeTracker::g_hContext = TX_EMPTY_HANDLE;
  TX_TICKET EyeTracker::g_hConnectionStateChangedTicket = TX_INVALID_TICKET;
  TX_TICKET EyeTracker::g_hEventHandlerTicket = TX_INVALID_TICKET;

  ros::Publisher EyeTracker::chatter_pub;
  eyetracker_talker::GazeData EyeTracker::last_data;
  eyetracker_talker::GazeData EyeTracker::data;
  bool EyeTracker::started = false;
  float EyeTracker::last_blink_diff = 0;
  unsigned char EyeTracker::consecutive_blinks = 0;


  int EyeTracker::run(int argc, char **argv) {
    bool success;

    ROS_INFO("EyeX: Initiating...");
    
    success = initEyeX();

    if (!success)
    {
      ROS_INFO("EyeX: Error initiating Tobii EyeX");
      return 1;
    }

    ROS_INFO("EyeX: Initiated");

    
    ROS_INFO("Initiating ROS node");
    
    initRosNode(argc, argv);

    ROS_INFO("Closing ROS node");
    
    closeRosNode();
    
    ROS_INFO("Closed ROS node");


    ROS_INFO("EyeX: Closing...");
    
    success = closeEyeX();

    if (!success)
    {
      ROS_INFO("EyeX: Could not shut down cleanly. Did you remember to release all handles?");
      return 2;
    }

    ROS_INFO("EyeX: Closed");

    // return success
    return 0;
  }

  /*
  * Application entry point.
  */
  bool EyeTracker::initEyeX() {
    bool success;

    // initialize and enable the context that is our link to the EyeX Engine.
    success = txInitializeEyeX(TX_EYEXCOMPONENTOVERRIDEFLAG_NONE, NULL, NULL, NULL, NULL) == TX_RESULT_OK;
    success &= txCreateContext(&g_hContext, TX_FALSE) == TX_RESULT_OK;
    success &= InitializeGlobalInteractorSnapshot(g_hContext);
    success &= txRegisterConnectionStateChangedHandler(g_hContext, &g_hConnectionStateChangedTicket, &EyeTracker::OnEngineConnectionStateChanged, NULL) == TX_RESULT_OK;
    success &= txRegisterEventHandler(g_hContext, &g_hEventHandlerTicket, &EyeTracker::HandleEvent, NULL) == TX_RESULT_OK;
    success &= txEnableConnection(g_hContext) == TX_RESULT_OK;

    // let the events flow until a key is pressed.
    return success;
  }

  bool EyeTracker::closeEyeX() {
    bool success;

    // disable and delete the context.
    txDisableConnection(g_hContext);
    txReleaseObject(&g_hGlobalInteractorSnapshot);
    success = txShutdownContext(g_hContext, TX_CLEANUPTIMEOUT_DEFAULT, TX_FALSE) == TX_RESULT_OK;
    success &= txReleaseContext(&g_hContext) == TX_RESULT_OK;
    success &= txUninitializeEyeX() == TX_RESULT_OK;

    return success;
  }

  void EyeTracker::initRosNode(int argc, char **argv) {
    ros::init(argc, argv, "eyetracker_talker");
    ros::NodeHandle n;
    chatter_pub = n.advertise<eyetracker_talker::GazeData>("eyetracker", 1000);
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void EyeTracker::closeRosNode()
  {

  }

  void EyeTracker::createGazeMessage(float x, float y, float ts)
  {
    float time_diff = 0;

    // prepare the current data
    data.timestamp = ts;
    data.loc.x = x;
    data.loc.y = y;
    data.event = eyetracker_talker::GazeData::EVENT_NONE;
    

    time_diff = data.timestamp - last_data.timestamp;
    ROS_INFO("Time difference: %.0f", time_diff);

    // Determine if there was a blink and its type
    if (time_diff >= 100 && time_diff < 300)
    {
      switch (consecutive_blinks)
      {
        case 1:
          data.event = eyetracker_talker::GazeData::EVENT_BLINK_CUSTOM_2X;
          ROS_INFO("Event: EVENT_BLINK_CUSTOM_2X");
          break;
        case 2:
          data.event = eyetracker_talker::GazeData::EVENT_BLINK_CUSTOM_3X;
          ROS_INFO("Event: EVENT_BLINK_CUSTOM_3X");
          break;
        default:
          data.event = eyetracker_talker::GazeData::EVENT_BLINK_CUSTOM;
          ROS_INFO("Event: EVENT_BLINK_CUSTOM");
          break;
      }
    }
    else if (time_diff >= 800 && time_diff < 1400)
    {
      data.event = eyetracker_talker::GazeData::EVENT_BLINK_1S;
      ROS_INFO("Event: EVENT_BLINK_1S");
    }
    else if (time_diff > 1400 && time_diff < 2000)
    {
      data.event = eyetracker_talker::GazeData::EVENT_BLINK_2S;
      ROS_INFO("Event: EVENT_BLINK_2S");
    }

    if (data.event != eyetracker_talker::GazeData::EVENT_NONE)
    {
      last_blink_diff = 0;
      consecutive_blinks = ++consecutive_blinks % 3;
    }
    else
    {
      last_blink_diff += time_diff;

      if (last_blink_diff > 100)
      {
        last_blink_diff = 0;
        consecutive_blinks = 0;
      }
    }

    // store the previous last data
    last_data = data;
  }

  void EyeTracker::sendGazeMessage()
  {
    if (!ros::ok())
    {
      return;
    }

    ROS_INFO("(%.1f, %.1f, %d)", data.loc.x, data.loc.y, data.event);

    chatter_pub.publish(data);
  }

  /*
   * Handles an event from the Gaze Point data stream.
   */
  void EyeTracker::OnGazeDataEvent(TX_HANDLE hGazeDataBehavior)
  {
    TX_GAZEPOINTDATAEVENTPARAMS eventParams;

    if (txGetGazePointDataEventParams(hGazeDataBehavior, &eventParams) == TX_RESULT_OK)
    {
      createGazeMessage(eventParams.X, eventParams.Y, eventParams.Timestamp);
      sendGazeMessage();
    }
    else
    {
      ROS_INFO("Failed to interpret gaze data event packet");
    }
  }

  /*
   * Initializes g_hGlobalInteractorSnapshot with an interactor that has the Gaze Point behavior.
   */
  bool EyeTracker::InitializeGlobalInteractorSnapshot(TX_CONTEXTHANDLE hContext)
  {
    TX_HANDLE hInteractor = TX_EMPTY_HANDLE;
    TX_GAZEPOINTDATAPARAMS params = { TX_GAZEPOINTDATAMODE_LIGHTLYFILTERED };
    bool success;

    success = txCreateGlobalInteractorSnapshot(
      hContext,
      InteractorId,
      &g_hGlobalInteractorSnapshot,
      &hInteractor) == TX_RESULT_OK;
    success &= txCreateGazePointDataBehavior(hInteractor, &params) == TX_RESULT_OK;

    txReleaseObject(&hInteractor);

    return success;
  }

  /*
   * Callback function invoked when a snapshot has been committed.
   */
  void TX_CALLCONVENTION EyeTracker::OnSnapshotCommitted(TX_CONSTHANDLE hAsyncData, TX_USERPARAM param)
  {
    // check the result code using an assertion.
    // this will catch validation errors and runtime errors in debug builds. in release builds it won't do anything.

    TX_RESULT result = TX_RESULT_UNKNOWN;
    txGetAsyncDataResultCode(hAsyncData, &result);
    assert(result == TX_RESULT_OK || result == TX_RESULT_CANCELLED);
  }

  /*
   * Callback function invoked when the status of the connection to the EyeX Engine has changed.
   */
  void TX_CALLCONVENTION EyeTracker::OnEngineConnectionStateChanged(TX_CONNECTIONSTATE connectionState, TX_USERPARAM userParam)
  {
    switch (connectionState) {
      case TX_CONNECTIONSTATE_CONNECTED: {
        BOOL success;
        ROS_INFO("EyeX: Connected to the EyeX Engine");
        // commit the snapshot with the global interactor as soon as the connection to the engine is established.
        // (it cannot be done earlier because committing means "send to the engine".)
        success = txCommitSnapshotAsync(g_hGlobalInteractorSnapshot, &EyeTracker::OnSnapshotCommitted, NULL) == TX_RESULT_OK;
        if (!success)
        {
          ROS_INFO("EyeX: Failed to initialize the data stream");
        }
        else
        {
          ROS_INFO("EyeX: Waiting for gaze data to stream...");
        }
      }
      break;

      case TX_CONNECTIONSTATE_DISCONNECTED:
      ROS_WARN("EyeX: We are disconnected from the EyeX Engine");
      break;

      case TX_CONNECTIONSTATE_TRYINGTOCONNECT:
      ROS_INFO("EyeX: Trying to connect to the EyeX Engine");
      break;

      case TX_CONNECTIONSTATE_SERVERVERSIONTOOLOW:
      ROS_ERROR("EyeX: This application requires a more recent version of the EyeX Engine to run");
      break;

      case TX_CONNECTIONSTATE_SERVERVERSIONTOOHIGH:
      ROS_ERROR("EyeX: This application requires an older version of the EyeX Engine to run");
      break;
    }
  }

  /*
   * Callback function invoked when an event has been received from the EyeX Engine.
   */
  void TX_CALLCONVENTION EyeTracker::HandleEvent(TX_CONSTHANDLE hAsyncData, TX_USERPARAM userParam)
  {
    TX_HANDLE hEvent = TX_EMPTY_HANDLE;
    TX_HANDLE hBehavior = TX_EMPTY_HANDLE;

    txGetAsyncDataContent(hAsyncData, &hEvent);

    // NOTE. Uncomment the following line of code to view the event object. The same function can be used with any interaction object.
    //OutputDebugStringA(txDebugObject(hEvent));

    if (txGetEventBehavior(hEvent, &hBehavior, TX_BEHAVIORTYPE_GAZEPOINTDATA) == TX_RESULT_OK)
    {
      OnGazeDataEvent(hBehavior);
      txReleaseObject(&hBehavior);
    }

    // NOTE since this is a very simple application with a single interactor and a single data stream, 
    // our event handling code can be very simple too. A more complex application would typically have to 
    // check for multiple behaviors and route events based on interactor IDs.

    txReleaseObject(&hEvent);
  }
}