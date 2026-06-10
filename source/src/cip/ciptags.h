/*******************************************************************************
 * DoXOM PLC Simulator - Tag Database for Allen Bradley emulation
 *
 * Provides named tag storage that maps symbolic CIP paths to data buffers.
 * Tags are registered with name, data type, and size - matching what our
 * ESP32 PLC firmware expects when using CIP Read Tag (0x4C) / Write Tag (0x4D)
 * with symbolic path segments (0x91 + name).
 ******************************************************************************/
#ifndef OPENER_CIPTAGS_H_
#define OPENER_CIPTAGS_H_

#include "typedefs.h"
#include "ciptypes.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of tags in the database */
#define CIP_TAG_DB_MAX_TAGS     64
/** Maximum length of a tag name */
#define CIP_TAG_DB_MAX_NAME_LEN 82

/** CIP data types matching our ESP32 firmware's eip_esp32.h definitions */
#define CIP_SIM_TYPE_BOOL       0xC1
#define CIP_SIM_TYPE_SINT       0xC2
#define CIP_SIM_TYPE_INT        0xC3
#define CIP_SIM_TYPE_DINT       0xC4
#define CIP_SIM_TYPE_LINT       0xC5
#define CIP_SIM_TYPE_USINT      0xC6
#define CIP_SIM_TYPE_UINT       0xC7
#define CIP_SIM_TYPE_UDINT      0xC8
#define CIP_SIM_TYPE_LWORD      0xC9
#define CIP_SIM_TYPE_REAL       0xCA
#define CIP_SIM_TYPE_LREAL      0xCB
#define CIP_SIM_TYPE_STRING     0xDA
#define CIP_SIM_TYPE_BYTE       0xD1
#define CIP_SIM_TYPE_WORD       0xD2
#define CIP_SIM_TYPE_DWORD      0xD3

/** Tag entry in the database */
typedef struct cip_tag_entry {
    char        name[CIP_TAG_DB_MAX_NAME_LEN]; /**< Tag name (case-insensitive lookup) */
    EipUint8    data_type;    /**< CIP data type code */
    size_t      data_size;    /**< Size in bytes of one element */
    EipUint16   array_count;  /**< Number of array elements (1 for scalar) */
    void       *data;         /**< Pointer to the data buffer */
    bool        in_use;       /**< Whether this slot is in use */
} CipTagEntry;

/** Get the number of bytes for a CIP data type */
size_t CipTagGetDataTypeSize(EipUint8 cip_type);

/** Initialize the tag database */
void CipTagDatabaseInit(void);

/** Register a tag with a name, type, and data pointer.
 *  Returns true on success, false if database is full. */
bool CipTagRegister(const char *name, EipUint8 data_type,
                    EipUint16 array_count, void *data);

/** Find a tag by name (case-insensitive). Returns NULL if not found. */
CipTagEntry *CipTagFind(const char *name);

/** Encode tag data into the ENIP message buffer for Read Tag response.
 *  Returns number of bytes written, or 0 on error. */
size_t CipTagEncodeData(const CipTagEntry *tag, ENIPMessage *outgoing_message);

/** Handle CIP Read Tag (0x4C) and Write Tag (0x4D) requests with symbolic paths.
 *  Parses the symbolic path segment (0x91 + name), looks up the tag in the
 *  database, and fills the response accordingly.
 *  Returns kEipStatusOkSend if the response was filled, kEipStatusError if
 *  the tag was not found (caller should fall through to normal processing). */
EipStatus CipTagHandleReadWrite(EipUint8 service_code,
                                const EipUint8 *path_data,
                                size_t path_data_length,
                                CipMessageRouterResponse *response);

#ifdef __cplusplus
}
#endif

#endif /* OPENER_CIPTAGS_H_ */