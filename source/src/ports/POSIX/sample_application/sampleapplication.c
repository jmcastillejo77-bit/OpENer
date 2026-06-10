/*******************************************************************************
 * Copyright (c) 2012, Rockwell Automation, Inc.
 * All rights reserved.
 *
 ******************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "opener_api.h"
#include "appcontype.h"
#include "trace.h"
#include "cipidentity.h"
#include "ciptcpipinterface.h"
#include "cipqos.h"
#include "nvdata.h"
#include "ciptags.h"
#if defined(OPENER_ETHLINK_CNTRS_ENABLE) && 0 != OPENER_ETHLINK_CNTRS_ENABLE
  #include "cipethernetlink.h"
  #include "ethlinkcbs.h"
#endif

#define DEMO_APP_INPUT_ASSEMBLY_NUM                100 //0x064
#define DEMO_APP_OUTPUT_ASSEMBLY_NUM               150 //0x096
#define DEMO_APP_CONFIG_ASSEMBLY_NUM               151 //0x097
#define DEMO_APP_HEARTBEAT_INPUT_ONLY_ASSEMBLY_NUM  152 //0x098
#define DEMO_APP_HEARTBEAT_LISTEN_ONLY_ASSEMBLY_NUM 153 //0x099
#define DEMO_APP_EXPLICT_ASSEMBLY_NUM              154 //0x09A

/* global variables for demo application (4 assembly data fields)  ************/

EipUint8 g_assembly_data064[32]; /* Input */
EipUint8 g_assembly_data096[32]; /* Output */
EipUint8 g_assembly_data097[10]; /* Config */
EipUint8 g_assembly_data09A[32]; /* Explicit */

/* DoXOM: tag storage for Allen Bradley simulation */
static CipBool   g_tag_StartButton       = true;
static CipBool   g_tag_StopButton        = false;
static CipBool   g_tag_MotorRun          = false;
static CipBool   g_tag_MotorFault        = false;
static CipBool   g_tag_Sensor1           = true;
static CipBool   g_tag_Sensor2           = false;
static CipBool   g_tag_Valve1            = false;
static CipBool   g_tag_LampGreen         = false;
static CipBool   g_tag_LampRed           = true;
static CipBool   g_tag_ModeAuto          = true;
static CipBool   g_tag_SystemReady       = true;
static CipBool   g_tag_EmergencyStop     = false;
static CipDint   g_tag_Counter1          = 42;
static CipDint   g_tag_Counter2          = 100;
static CipDint   g_tag_Timer_Preset      = 5000;
static CipDint   g_tag_Timer_Accum       = 0;
static CipDint   g_tag_ProductionCount   = 1234;
static CipDint   g_tag_ErrorCode         = 0;
static CipDint   g_tag_SetPoint_Temperature = 250;
static CipInt    g_tag_SpeedRPM          = 1500;
static CipInt    g_tag_Temperature       = 25;
static CipInt    g_tag_Pressure          = 100;
static CipReal   g_tag_FlowRate          = 1.5f;
static CipReal   g_tag_Level             = 75.0f;
static CipReal   g_tag_PID_Output        = 50.0f;
static CipReal   g_tag_PID_Setpoint      = 100.0f;
static CipReal   g_tag_Voltage           = 24.0f;
static CipSint   g_tag_AlarmLevel        = 3;
static CipSint   g_tag_Percentage        = 75;
static CipSint   g_tag_Mode              = 1;
static CipUint   g_tag_StatusWord        = 0x0001;
static CipUint   g_tag_ControlWord       = 0x0000;
static CipUdint  g_tag_Runtime           = 3600;
static CipUdint  g_tag_TotalCycles       = 500000;
static CipDint   g_tag_ArrayData[10]     = {0,1,2,3,4,5,6,7,8,9};
static CipInt    g_tag_AnalogInputs[5]   = {0,512,1024,2048,4095};

/* local functions */

/* global functions called by the stack */
EipStatus ApplicationInitialization(void) {
  /* create 3 assembly object instances*/
  /*INPUT*/
  CreateAssemblyObject( DEMO_APP_INPUT_ASSEMBLY_NUM, g_assembly_data064,
                        sizeof(g_assembly_data064) );

  /*OUTPUT*/
  CreateAssemblyObject( DEMO_APP_OUTPUT_ASSEMBLY_NUM, g_assembly_data096,
                        sizeof(g_assembly_data096) );

  /*CONFIG*/
  CreateAssemblyObject( DEMO_APP_CONFIG_ASSEMBLY_NUM, g_assembly_data097,
                        sizeof(g_assembly_data097) );

  /*Heart-beat output assembly for Input only connections */
  CreateAssemblyObject(DEMO_APP_HEARTBEAT_INPUT_ONLY_ASSEMBLY_NUM, NULL, 0);

  /*Heart-beat output assembly for Listen only connections */
  CreateAssemblyObject(DEMO_APP_HEARTBEAT_LISTEN_ONLY_ASSEMBLY_NUM, NULL, 0);

  /* assembly for explicit messaging */
  CreateAssemblyObject( DEMO_APP_EXPLICT_ASSEMBLY_NUM, g_assembly_data09A,
                        sizeof(g_assembly_data09A) );

  ConfigureExclusiveOwnerConnectionPoint(0, DEMO_APP_OUTPUT_ASSEMBLY_NUM,
                                         DEMO_APP_INPUT_ASSEMBLY_NUM,
                                         DEMO_APP_CONFIG_ASSEMBLY_NUM);
  ConfigureInputOnlyConnectionPoint(0,
                                    DEMO_APP_HEARTBEAT_INPUT_ONLY_ASSEMBLY_NUM,
                                    DEMO_APP_INPUT_ASSEMBLY_NUM,
                                    DEMO_APP_CONFIG_ASSEMBLY_NUM);
  ConfigureListenOnlyConnectionPoint(0,
                                     DEMO_APP_HEARTBEAT_LISTEN_ONLY_ASSEMBLY_NUM,
                                     DEMO_APP_INPUT_ASSEMBLY_NUM,
                                     DEMO_APP_CONFIG_ASSEMBLY_NUM);

  /* DoXOM: Register tags in tag database for CIP Read/Write Tag */
  CipTagDatabaseInit();
  CipTagRegister("StartButton",        CIP_SIM_TYPE_BOOL,  1, &g_tag_StartButton);
  CipTagRegister("StopButton",         CIP_SIM_TYPE_BOOL,  1, &g_tag_StopButton);
  CipTagRegister("MotorRun",           CIP_SIM_TYPE_BOOL,  1, &g_tag_MotorRun);
  CipTagRegister("MotorFault",         CIP_SIM_TYPE_BOOL,  1, &g_tag_MotorFault);
  CipTagRegister("Sensor1",            CIP_SIM_TYPE_BOOL,  1, &g_tag_Sensor1);
  CipTagRegister("Sensor2",            CIP_SIM_TYPE_BOOL,  1, &g_tag_Sensor2);
  CipTagRegister("Valve1",             CIP_SIM_TYPE_BOOL,  1, &g_tag_Valve1);
  CipTagRegister("LampGreen",          CIP_SIM_TYPE_BOOL,  1, &g_tag_LampGreen);
  CipTagRegister("LampRed",            CIP_SIM_TYPE_BOOL,  1, &g_tag_LampRed);
  CipTagRegister("ModeAuto",           CIP_SIM_TYPE_BOOL,  1, &g_tag_ModeAuto);
  CipTagRegister("SystemReady",        CIP_SIM_TYPE_BOOL,  1, &g_tag_SystemReady);
  CipTagRegister("EmergencyStop",      CIP_SIM_TYPE_BOOL,  1, &g_tag_EmergencyStop);
  CipTagRegister("Counter1",           CIP_SIM_TYPE_DINT,  1, &g_tag_Counter1);
  CipTagRegister("Counter2",           CIP_SIM_TYPE_DINT,  1, &g_tag_Counter2);
  CipTagRegister("Timer_Preset",       CIP_SIM_TYPE_DINT,  1, &g_tag_Timer_Preset);
  CipTagRegister("Timer_Accum",        CIP_SIM_TYPE_DINT,  1, &g_tag_Timer_Accum);
  CipTagRegister("ProductionCount",    CIP_SIM_TYPE_DINT,  1, &g_tag_ProductionCount);
  CipTagRegister("ErrorCode",          CIP_SIM_TYPE_DINT,  1, &g_tag_ErrorCode);
  CipTagRegister("SetPoint_Temperature", CIP_SIM_TYPE_DINT, 1, &g_tag_SetPoint_Temperature);
  CipTagRegister("SpeedRPM",           CIP_SIM_TYPE_INT,   1, &g_tag_SpeedRPM);
  CipTagRegister("Temperature",        CIP_SIM_TYPE_INT,   1, &g_tag_Temperature);
  CipTagRegister("Pressure",           CIP_SIM_TYPE_INT,   1, &g_tag_Pressure);
  CipTagRegister("FlowRate",           CIP_SIM_TYPE_REAL,  1, &g_tag_FlowRate);
  CipTagRegister("Level",              CIP_SIM_TYPE_REAL,  1, &g_tag_Level);
  CipTagRegister("PID_Output",         CIP_SIM_TYPE_REAL,  1, &g_tag_PID_Output);
  CipTagRegister("PID_Setpoint",       CIP_SIM_TYPE_REAL,  1, &g_tag_PID_Setpoint);
  CipTagRegister("Voltage",            CIP_SIM_TYPE_REAL,  1, &g_tag_Voltage);
  CipTagRegister("AlarmLevel",         CIP_SIM_TYPE_SINT,  1, &g_tag_AlarmLevel);
  CipTagRegister("Percentage",         CIP_SIM_TYPE_SINT,  1, &g_tag_Percentage);
  CipTagRegister("Mode",               CIP_SIM_TYPE_SINT,  1, &g_tag_Mode);
  CipTagRegister("StatusWord",         CIP_SIM_TYPE_UINT,  1, &g_tag_StatusWord);
  CipTagRegister("ControlWord",        CIP_SIM_TYPE_UINT,  1, &g_tag_ControlWord);
  CipTagRegister("Runtime",            CIP_SIM_TYPE_UDINT, 1, &g_tag_Runtime);
  CipTagRegister("TotalCycles",        CIP_SIM_TYPE_UDINT, 1, &g_tag_TotalCycles);
  CipTagRegister("ArrayData",          CIP_SIM_TYPE_DINT, 10, g_tag_ArrayData);
  CipTagRegister("AnalogInputs",       CIP_SIM_TYPE_INT,   5, g_tag_AnalogInputs);

  /* For NV data support connect callback functions for each object class with
   *  NV data.
   */
  InsertGetSetCallback(GetCipClass(kCipQoSClassCode), NvQosSetCallback,
                       kNvDataFunc);
  InsertGetSetCallback(GetCipClass(kCipTcpIpInterfaceClassCode),
                       NvTcpipSetCallback,
                       kNvDataFunc);

#if defined(OPENER_ETHLINK_CNTRS_ENABLE) && 0 != OPENER_ETHLINK_CNTRS_ENABLE
  /* For the Ethernet Interface & Media Counters connect a PreGetCallback and
   *  a PostGetCallback.
   * The PreGetCallback is used to fetch the counters from the hardware.
   * The PostGetCallback is utilized by the GetAndClear service to clear
   *  the hardware counters after the current data have been transmitted.
   */
  {
    CipClass *p_eth_link_class = GetCipClass(kCipEthernetLinkClassCode);
    InsertGetSetCallback(p_eth_link_class,
                         EthLnkPreGetCallback,
                         kPreGetFunc);
    InsertGetSetCallback(p_eth_link_class,
                         EthLnkPostGetCallback,
                         kPostGetFunc);
    /* Specify the attributes for which the callback should be executed. */
    for (int idx = 0; idx < OPENER_ETHLINK_INSTANCE_CNT; ++idx)
    {
      CipAttributeStruct *p_eth_link_attr;
      CipInstance *p_eth_link_inst =
        GetCipInstance(p_eth_link_class, idx + 1);
      OPENER_ASSERT(p_eth_link_inst);

      /* Interface counters attribute */
      p_eth_link_attr = GetCipAttribute(p_eth_link_inst, 4);
      p_eth_link_attr->attribute_flags |= (kPreGetFunc | kPostGetFunc);
      /* Media counters attribute */
      p_eth_link_attr = GetCipAttribute(p_eth_link_inst, 5);
      p_eth_link_attr->attribute_flags |= (kPreGetFunc | kPostGetFunc);
    }
  }
#endif

  return kEipStatusOk;
}

void HandleApplication(void) {
  /* check if application needs to trigger an connection */
}

void CheckIoConnectionEvent(unsigned int output_assembly_id,
                            unsigned int input_assembly_id,
                            IoConnectionEvent io_connection_event) {
  /* maintain a correct output state according to the connection state*/

  (void) output_assembly_id; /* suppress compiler warning */
  (void) input_assembly_id; /* suppress compiler warning */
  (void) io_connection_event; /* suppress compiler warning */
}

EipStatus AfterAssemblyDataReceived(CipInstance *instance) {
  EipStatus status = kEipStatusOk;

  /*handle the data received e.g., update outputs of the device */
  switch (instance->instance_number) {
    case DEMO_APP_OUTPUT_ASSEMBLY_NUM:
      /* Data for the output assembly has been received.
       * Mirror it to the inputs */
      memcpy( &g_assembly_data064[0], &g_assembly_data096[0],
              sizeof(g_assembly_data064) );
      break;
    case DEMO_APP_EXPLICT_ASSEMBLY_NUM:
      /* do something interesting with the new data from
       * the explicit set-data-attribute message */
      break;
    case DEMO_APP_CONFIG_ASSEMBLY_NUM:
      /* Add here code to handle configuration data and check if it is ok
       * The demo application does not handle config data.
       * However in order to pass the test we accept any data given.
       * EIP_ERROR
       */
      status = kEipStatusOk;
      break;
    default:
      OPENER_TRACE_INFO(
        "Unknown assembly instance ind AfterAssemblyDataReceived");
      break;
  }
  return status;
}

EipBool8 BeforeAssemblyDataSend(CipInstance *pa_pstInstance) {
  /*update data to be sent e.g., read inputs of the device */
  /*In this sample app we mirror the data from out to inputs on data receive
   * therefore we need nothing to do here. Just return true to inform that
   * the data is new.
   */

  if (pa_pstInstance->instance_number == DEMO_APP_EXPLICT_ASSEMBLY_NUM) {
    /* do something interesting with the existing data
     * for the explicit get-data-attribute message */
  }
  return true;
}

EipStatus ResetDevice(void) {
  /* add reset code here*/
  CloseAllConnections();
  CipQosUpdateUsedSetQosValues();
  return kEipStatusOk;
}

EipStatus ResetDeviceToInitialConfiguration(void) {
  /*rest the parameters */
  g_tcpip.encapsulation_inactivity_timeout = 120;
  CipQosResetAttributesToDefaultValues();
  /*than perform device reset*/
  ResetDevice();
  return kEipStatusOk;
}

void *
CipCalloc(size_t number_of_elements,
          size_t size_of_element) {
  return calloc(number_of_elements, size_of_element);
}

void CipFree(void *data) {
  free(data);
}

void RunIdleChanged(EipUint32 run_idle_value) {
  OPENER_TRACE_INFO("Run/Idle handler triggered\n");
  if( (0x0001 & run_idle_value) == 1 ) {
    CipIdentitySetExtendedDeviceStatus(kAtLeastOneIoConnectionInRunMode);
  } else {
    CipIdentitySetExtendedDeviceStatus(
      kAtLeastOneIoConnectionEstablishedAllInIdleMode);
  }
  (void) run_idle_value;
}