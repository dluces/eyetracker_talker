/**
 * @file eyetracker_talker/src/main.cpp
 *
 * @brief Test Program
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/eyetracker_talker/eyetracker_talker.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  // Run the talker
  return eyetracker_talker::EyeTracker::run(argc, argv);
}
