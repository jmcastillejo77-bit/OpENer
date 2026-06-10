/*******************************************************************************
 * Copyright (c) 2009, Rockwell Automation, Inc.
 * All rights reserved.
 *
 * DoXOM modifications: intercept Forward Open (0x54) and Read/Write Tag
 * services (0x4C/0x4D/0x4E) with symbolic paths (0x91 + name).
 ******************************************************************************/
#include <string.h>
#include "opener_api.h"
#include "cipcommon.h"
#include "endianconv.h"
#include "ciperror.h"
#include "trace.h"
#include "enipmessage.h"
#include "ciptags.h"

#include "cipmessagerouter.h"

#define CIP_SERVICE_READ_TAG       0x4C
#define CIP_SERVICE_WRITE_TAG      0x4D
#define CIP_SERVICE_READ_MODIFY    0x4E

CipMessageRouterRequest g_message_router_request;

/** @brief A class registry list node */
typedef struct cip_message_router_object {
  struct cip_message_router_object *next;
  CipClass *cip_class;
} CipMessageRouterObject;

CipMessageRouterObject *g_first_object = NULL;

EipStatus RegisterCipClass(CipClass *cip_class);
CipError CreateMessageRouterRequestStructure(const EipUint8 *data,
                                             EipInt16 data_length,
                                             CipMessageRouterRequest *message_router_request);

void InitializeCipMessageRouterClass(CipClass *cip_class) {
  CipClass *meta_class = cip_class->class_instance.cip_class;
  InsertAttribute( (CipInstance *) cip_class, 1, kCipUint, EncodeCipUint, NULL,
                   (void *) &cip_class->revision, kGetableSingleAndAll );
  InsertAttribute( (CipInstance *) cip_class, 2, kCipUint, EncodeCipUint, NULL,
                   (void *) &cip_class->number_of_instances, kGetableSingle );
  InsertAttribute( (CipInstance *) cip_class, 3, kCipUint, EncodeCipUint, NULL,
                   (void *) &cip_class->number_of_instances, kGetableSingle );
  InsertAttribute( (CipInstance *) cip_class, 4, kCipUint, EncodeCipUint, NULL,
                   (void *) &kCipUintZero, kGetableAll );
  InsertAttribute( (CipInstance *) cip_class, 5, kCipUint, EncodeCipUint, NULL,
                   (void *) &kCipUintZero, kGetableAll );
  InsertAttribute( (CipInstance *) cip_class, 6, kCipUint, EncodeCipUint, NULL,
                   (void *) &meta_class->highest_attribute_number,
                   kGetableSingleAndAll );
  InsertAttribute( (CipInstance *) cip_class, 7, kCipUint, EncodeCipUint, NULL,
                   (void *) &cip_class->highest_attribute_number,
                   kGetableSingleAndAll );
  InsertService(meta_class, kGetAttributeAll, &GetAttributeAll, "GetAttributeAll");
  InsertService(meta_class, kGetAttributeSingle, &GetAttributeSingle, "GetAttributeSingle");
}

EipStatus CipMessageRouterInit() {
  CipClass *message_router = CreateCipClass(kCipMessageRouterClassCode,
      7, 7, 2, 0, 0, 1, 1, "message router", 1, InitializeCipMessageRouterClass);
  if(NULL == message_router) return kEipStatusError;
  InsertService(message_router, kGetAttributeSingle, &GetAttributeSingle, "GetAttributeSingle");
  return kEipStatusOk;
}

CipMessageRouterObject *GetRegisteredObject(EipUint32 class_id) {
  CipMessageRouterObject *object = g_first_object;
  while(NULL != object) {
    OPENER_ASSERT(NULL != object->cip_class);
    if(object->cip_class->class_code == class_id) return object;
    object = object->next;
  }
  return NULL;
}

CipClass *GetCipClass(const CipUdint class_code) {
  CipMessageRouterObject *obj = GetRegisteredObject(class_code);
  return obj ? obj->cip_class : NULL;
}

CipInstance *GetCipInstance(const CipClass *RESTRICT const cip_class,
                            const CipInstanceNum instance_number) {
  if(instance_number == 0) return (CipInstance *) cip_class;
  for(CipInstance *instance = cip_class->instances; instance; instance = instance->next)
    if(instance->instance_number == instance_number) return instance;
  return NULL;
}

EipStatus RegisterCipClass(CipClass *cip_class) {
  CipMessageRouterObject **obj = &g_first_object;
  while(*obj) obj = &(*obj)->next;
  *obj = (CipMessageRouterObject *) CipCalloc(1, sizeof(CipMessageRouterObject));
  if(!*obj) return kEipStatusError;
  (*obj)->cip_class = cip_class;
  (*obj)->next = NULL;
  return kEipStatusOk;
}

EipStatus NotifyMessageRouter(EipUint8 *data,
                              int data_length,
                              CipMessageRouterResponse *message_router_response,
                              const struct sockaddr *const originator_address,
                              const CipSessionHandle encapsulation_session) {
  EipStatus eip_status = kEipStatusOkSend;
  CipError status = kCipErrorSuccess;
  EipUint8 service_code = *data;

  OPENER_TRACE_INFO("NotifyMessageRouter: routing unconnected message\n");

  /* DoXOM FAST PATH: Forward Open (0x54) - respond with fake success.
   * This allows the PLC_H7_RV firmware to proceed past Forward Open
   * without OpENer needing to decode the Rockwell connection path. */
  if(service_code == 0x54) {
    OPENER_TRACE_INFO("DoXOM: Forward Open intercepted\n");
    message_router_response->reply_service = (0x80 | service_code);
    message_router_response->general_status = 0x00;
    message_router_response->size_of_additional_status = 0;
    message_router_response->reserved = 0;
    {
      EipUint8 *buf = message_router_response->message.message_buffer;
      EipUint32 ot = 0x12345678, to = 0x87654321;
      memcpy(buf, &ot, 4);
      memcpy(buf+4, &to, 4);
      buf[8] = 1; buf[9] = 0;
      message_router_response->message.used_message_length = 10;
    }
    return kEipStatusOkSend;
  }

  /* DoXOM FAST PATH: Read/Write Tag with symbolic path (0x91 + name) */
  if(service_code == CIP_SERVICE_READ_TAG || service_code == CIP_SERVICE_WRITE_TAG ||
     service_code == CIP_SERVICE_READ_MODIFY) {
    if(data_length > 2 && data[1] == 0x91) {
      EipStatus tag_result = CipTagHandleReadWrite(service_code, data + 1,
        data_length - 1, message_router_response);
      if(tag_result == kEipStatusOkSend) {
        message_router_response->reply_service = (0x80 | service_code);
        return kEipStatusOkSend;
      }
    }
  }

  /* Standard CIP processing */
  if(kCipErrorSuccess !=
     (status = CreateMessageRouterRequestStructure(data, data_length, &g_message_router_request))) {
    OPENER_TRACE_ERR("NotifyMessageRouter: error from createMRRequeststructure\n");
    message_router_response->general_status = status;
    message_router_response->size_of_additional_status = 0;
    message_router_response->reserved = 0;
    message_router_response->reply_service = (0x80 | g_message_router_request.service);
  } else {
    CipMessageRouterObject *registered_object = GetRegisteredObject(g_message_router_request.request_path.class_id);
    if(!registered_object) {
      OPENER_TRACE_ERR("NotifyMessageRouter: class 0x%x not registered\n",
        (unsigned)g_message_router_request.request_path.class_id);
      message_router_response->general_status = kCipErrorPathDestinationUnknown;
      message_router_response->size_of_additional_status = 0;
      message_router_response->reserved = 0;
      message_router_response->reply_service = (0x80 | g_message_router_request.service);
    } else {
      message_router_response->reserved = 0;
      eip_status = NotifyClass(registered_object->cip_class, &g_message_router_request,
        message_router_response, originator_address, encapsulation_session);
    }
  }
  return eip_status;
}

CipError CreateMessageRouterRequestStructure(const EipUint8 *data,
                                             EipInt16 data_length,
                                             CipMessageRouterRequest *mr) {
  mr->service = *data;
  data++;
  data_length--;
  size_t decoded;
  if(DecodePaddedEPath(&(mr->request_path), &data, &decoded) != kEipStatusOk)
    return kCipErrorPathSegmentError;
  if(decoded > data_length) return kCipErrorPathSizeInvalid;
  mr->data = data;
  mr->request_data_size = data_length - decoded;
  return kCipErrorSuccess;
}

void DeleteAllClasses(void) {
  CipMessageRouterObject *obj = g_first_object;
  while(obj) {
    CipMessageRouterObject *del = obj;
    obj = obj->next;
    CipInstance *inst = del->cip_class->instances;
    while(inst) { CipInstance *id = inst; inst = inst->next; CipFree(id->attributes); CipFree(id); }
    CipClass *meta = del->cip_class->class_instance.cip_class;
    CipFree(meta->class_name); CipFree(meta->services); CipFree(meta->get_single_bit_mask);
    CipFree(meta->set_bit_mask); CipFree(meta->get_all_bit_mask); CipFree(meta);
    CipClass *cc = del->cip_class;
    CipFree(cc->class_name); CipFree(cc->get_single_bit_mask); CipFree(cc->set_bit_mask);
    CipFree(cc->get_all_bit_mask); CipFree(cc->class_instance.attributes); CipFree(cc->services);
    CipFree(cc); CipFree(del);
  }
  g_first_object = NULL;
}