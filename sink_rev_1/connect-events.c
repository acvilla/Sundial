// This file is generated by Ember Desktop.  Please do not edit manually.
//
//

// Enclosing macro to prevent multiple inclusion
#ifndef __CONNECT_EVENTS__
#define __CONNECT_EVENTS__


// Generated events.
#include PLATFORM_HEADER
#include "stack/include/ember-types.h"
#include "hal/hal.h"


void advertiseHandler(void);
extern EmberEventControl advertiseControl;

void dataReportHandler(void);
extern EmberEventControl dataReportControl;

void emberAfPluginHeartbeatEventHandler(void);
extern EmberEventControl emberAfPluginHeartbeatEventControl;



const EmberEventData emAppEvents[] = {
  {&advertiseControl, advertiseHandler},
  {&dataReportControl, dataReportHandler},
  {&emberAfPluginHeartbeatEventControl, emberAfPluginHeartbeatEventHandler},
  {NULL, NULL}
};
#endif // __CONNECT_EVENTS__