/**
 * @file /include/eyetracker_talker/eyetracker_talker.hpp
 *
 * @brief Template class for eyetracker_talker.
 **/
#ifndef eyetracker_talker_HPP
#define eyetracker_talker_HPP

#ifdef WIN32
  #ifdef eyetracker_talker_EXPORTS
    #define eyetracker_talker_API __declspec(dllexport)
  #else
    #define eyetracker_talker_API __declspec(dllimport)
  #endif
#else
  #define eyetracker_talker_API
#endif

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <eyex\EyeX.h>

namespace eyetracker_talker
{
  /**
   * @brief Template class for $(package)s
   */
  class eyetracker_talker_API EyeTracker
  {

  public:
    // ID of the global interactor that provides our data stream; must be unique within the application.
    static const TX_STRING InteractorId;
    // global variables
    static TX_HANDLE g_hGlobalInteractorSnapshot;
    static TX_CONTEXTHANDLE g_hContext;
    static TX_TICKET g_hConnectionStateChangedTicket;
    static TX_TICKET g_hEventHandlerTicket;

    static geometry_msgs::Point32 point;
    static ros::Publisher chatter_pub;

    static int run(int, char **);
    static void initRosNode(int, char **);
    static bool initEyeX();
    static bool closeEyeX();
    static void closeRosNode();
    static void sendPoint(float, float);

    static void TX_CALLCONVENTION OnSnapshotCommitted(TX_CONSTHANDLE, TX_USERPARAM);
    static void TX_CALLCONVENTION OnEngineConnectionStateChanged(TX_CONNECTIONSTATE, TX_USERPARAM);
    static void TX_CALLCONVENTION HandleEvent(TX_CONSTHANDLE, TX_USERPARAM);

  private:
    EyeTracker() {};

    static bool InitializeGlobalInteractorSnapshot(TX_CONTEXTHANDLE);
    static void OnGazeDataEvent(TX_HANDLE);

  };

} // namespace eyetracker_talker

#endif // eyetracker_talker_HPP
