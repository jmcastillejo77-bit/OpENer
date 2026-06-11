/*******************************************************************************
 * DoXOM PLC Simulator - Tag Database Implementation
 *
 * Provides named tag storage for Allen Bradley PLC simulation.
 * Our ESP32 firmware uses CIP Read Tag (0x4C) / Write Tag (0x4D) with
 * symbolic path segments (0x91 + name). This module provides the backing
 * storage and handling for those named tags.
 ******************************************************************************/
#include "ciptags.h"
#include "endianconv.h"
#include "enipmessage.h"
#include "trace.h"
#include <string.h>
#include <ctype.h>

/** Global tag database */
static CipTagEntry g_tag_db[CIP_TAG_DB_MAX_TAGS];
static bool g_initialized = false;

/** Get the CIP data type size in bytes */
size_t CipTagGetDataTypeSize(EipUint8 cip_type) {
    switch(cip_type) {
        case CIP_SIM_TYPE_BOOL:
        case CIP_SIM_TYPE_SINT:
        case CIP_SIM_TYPE_USINT:
        case CIP_SIM_TYPE_BYTE:
            return 1;
        case CIP_SIM_TYPE_INT:
        case CIP_SIM_TYPE_UINT:
        case CIP_SIM_TYPE_WORD:
            return 2;
        case CIP_SIM_TYPE_DINT:
        case CIP_SIM_TYPE_UDINT:
        case CIP_SIM_TYPE_REAL:
        case CIP_SIM_TYPE_DWORD:
            return 4;
        case CIP_SIM_TYPE_LINT:
        case CIP_SIM_TYPE_LWORD:
        case CIP_SIM_TYPE_LREAL:
            return 8;
        case CIP_SIM_TYPE_STRING:
            return 88;  /* ControlLogix STRING: 4 bytes length + 82 chars + 2 pad */
        default:
            return 0;
    }
}

/** Case-insensitive string comparison */
static bool str_equal_nocase(const char *a, const char *b) {
    while(*a && *b) {
        if(tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return false;
        }
        a++;
        b++;
    }
    return (*a == *b);
}

/** Initialize the tag database */
void CipTagDatabaseInit(void) {
    memset(g_tag_db, 0, sizeof(g_tag_db));
    g_initialized = true;
    OPENER_TRACE_INFO("DoXOM Tag Database: initialized with %d slots\n",
                      CIP_TAG_DB_MAX_TAGS);
}

/** Register a tag */
bool CipTagRegister(const char *name, EipUint8 data_type,
                    EipUint16 array_count, void *data) {
    if(!g_initialized) {
        CipTagDatabaseInit();
    }

    if(!name || !data || array_count == 0) {
        return false;
    }

    size_t type_size = CipTagGetDataTypeSize(data_type);
    if(type_size == 0) {
        OPENER_TRACE_ERR("DoXOM Tag DB: unknown data type 0x%02X for tag '%s'\n",
                         data_type, name);
        return false;
    }

    /* Find empty slot */
    for(int i = 0; i < CIP_TAG_DB_MAX_TAGS; i++) {
        if(!g_tag_db[i].in_use) {
            strncpy(g_tag_db[i].name, name, CIP_TAG_DB_MAX_NAME_LEN - 1);
            g_tag_db[i].name[CIP_TAG_DB_MAX_NAME_LEN - 1] = '\0';
            g_tag_db[i].data_type = data_type;
            g_tag_db[i].data_size = type_size;
            g_tag_db[i].array_count = array_count;
            g_tag_db[i].data = data;
            g_tag_db[i].in_use = true;

            OPENER_TRACE_INFO("DoXOM Tag DB: registered '%s' type=0x%02X size=%zu count=%u\n",
                              name, data_type, type_size, array_count);
            return true;
        }
    }

    OPENER_TRACE_ERR("DoXOM Tag DB: database full, cannot register '%s'\n", name);
    return false;
}

/** Find a tag by name (case-insensitive) */
CipTagEntry *CipTagFind(const char *name) {
    if(!name || !g_initialized) {
        return NULL;
    }

    for(int i = 0; i < CIP_TAG_DB_MAX_TAGS; i++) {
        if(g_tag_db[i].in_use &&
           str_equal_nocase(g_tag_db[i].name, name)) {
            return &g_tag_db[i];
        }
    }
    return NULL;
}

/** Encode tag data into message buffer for Read Tag response.
 *  Format: data_type(2 bytes) + data */
size_t CipTagEncodeData(const CipTagEntry *tag, ENIPMessage *outgoing_message) {
    if(!tag || !outgoing_message) {
        return 0;
    }

    size_t total_size = tag->data_size * tag->array_count;

    /* Write data type */
    AddIntToMessage(tag->data_type, outgoing_message);

    /* Write data */
    memcpy(&(outgoing_message->message_buffer[outgoing_message->used_message_length]),
           tag->data, total_size);
    outgoing_message->used_message_length += total_size;

    return total_size + 2;  /* data + type word */
}

/** Handle CIP Read Tag (0x4C) and Write Tag (0x4D) with symbolic paths.
 *
 * The path_data points to the symbolic segment: 0x91 <len> <name_bytes> [pad]
 * followed by element count (for Read Tag) or type+count+data (for Write Tag).
 *
 * Our ESP32 firmware sends Read Tag as:
 *   Service: 0x4C
 *   Path:    0x91 <name_len> <name_bytes> [pad_to_even]
 *   Data:    <element_count: UINT16>
 *
 * And Write Tag as:
 *   Service: 0x4D
 *   Path:    0x91 <name_len> <name_bytes> [pad_to_even]
 *   Data:    <data_type: UINT16> <element_count: UINT16> <data_bytes>
 */
EipStatus CipTagHandleReadWrite(EipUint8 service_code,
                                const EipUint8 *path_data,
                                size_t path_data_length,
                                CipMessageRouterResponse *response) {
    if(!path_data || !response || path_data_length < 2) {
        return kEipStatusError;
    }

    /* Verify symbolic segment type (0x91) */
    if(path_data[0] != 0x91) {
        return kEipStatusError;
    }

    /* Extract tag name length */
    size_t name_len = path_data[1];
    if(name_len + 2 > path_data_length) {
        OPENER_TRACE_ERR("DoXOM Tag: name length %zu exceeds path data\n", name_len);
        return kEipStatusError;
    }

    /* FIX: Prevenir Buffer Overflow si el nombre enviado por la red es muy largo */
    if(name_len >= CIP_TAG_DB_MAX_NAME_LEN) {
        OPENER_TRACE_ERR("DoXOM Tag: tag name length (%zu) exceeds maximum allowed\n", name_len);
        return kEipStatusError;
    }

    /* Extract tag name (may not be null-terminated) */
    char tag_name[CIP_TAG_DB_MAX_NAME_LEN];
    memset(tag_name, 0, sizeof(tag_name));
    memcpy(tag_name, &path_data[2], name_len);
    tag_name[name_len] = '\0';

    /* Calculate path consumed: 0x91 + len_byte + name_bytes + optional_pad */
    size_t path_consumed = 2 + name_len;
    if(path_consumed % 2 != 0) {
        path_consumed++;  /* pad to even word boundary */
    }

    OPENER_TRACE_INFO("DoXOM Tag: %s tag '%s' (path_len=%zu hex:",
                      (service_code == 0x4C || service_code == 0x52) ? "Read" : "Write",
                      tag_name, name_len);
    for(size_t h = 0; h < name_len && h < 32; h++)
      OPENER_TRACE_INFO(" %02x", (unsigned)path_data[2 + h]);
    OPENER_TRACE_INFO(")\n");

    /* Look up tag in database */
    CipTagEntry *tag = CipTagFind(tag_name);
    if(!tag) {
        /* Tag not found: for Read Tag, return a default DINT(0) response
         * instead of an error.  This allows the PLC firmware to proceed
         * even when tag names use multi-segment paths we don't fully
         * decode. */
        OPENER_TRACE_INFO("DoXOM Tag: tag '%s' not found, returning default DINT=0\n", tag_name);
        response->message.message_buffer[0] = (0x80 | service_code);
        response->message.message_buffer[1] = 0x00;
        response->message.message_buffer[2] = 0x00;  /* general_status = success */
        response->message.message_buffer[3] = 0x00;  /* addl_status_size */
        /* CIP data type = DINT (0xC4) + 4 bytes of zero */
        response->message.message_buffer[4] = 0xC4;
        response->message.message_buffer[5] = 0x00;
        response->message.message_buffer[6] = 0x00;
        response->message.message_buffer[7] = 0x00;
        response->message.message_buffer[8] = 0x00;
        response->message.message_buffer[9] = 0x00;
        response->message.used_message_length = 10;
        response->message.current_message_position = response->message.message_buffer + 10;
        response->general_status = 0x00;
        response->size_of_additional_status = 0;
        return kEipStatusOkSend;
    }

    /* Pointer to request data after the path */
    const EipUint8 *req_data = path_data + path_consumed;

    if(service_code == 0x4C || service_code == 0x52) {
        /* Read Tag Service */
        /* Request data: element_count (UINT16) */
        EipUint16 element_count = req_data[0] | (req_data[1] << 8);
        if(element_count == 0) element_count = 1;

        OPENER_TRACE_INFO("DoXOM Tag: reading %u elements of '%s'\n",
                          element_count, tag_name);

        /* Build response in the message buffer */
        size_t total_size = tag->data_size * element_count;

        /* CIP header: reply_service + reserved + status + addl_size */
        response->message.message_buffer[0] = (uint8_t)(0x80 | service_code);
        response->message.message_buffer[1] = 0x00;
        response->message.message_buffer[2] = 0x00;
        response->message.message_buffer[3] = 0x00;
        response->message.used_message_length = 4;
        response->message.current_message_position = response->message.message_buffer + 4;

        /* Response data type */
        AddIntToMessage(tag->data_type, &response->message);
        /* Response data */
        for(EipUint16 i = 0; i < element_count; i++) {
            memcpy(&(response->message.message_buffer[response->message.used_message_length]),
                   (const uint8_t *)tag->data + (i * tag->data_size),
                   tag->data_size);
            response->message.used_message_length += tag->data_size;
        }

        response->general_status = 0x00; /* Success */
        response->size_of_additional_status = 0;

        OPENER_TRACE_INFO("DoXOM Tag: read %zu bytes from '%s'\n",
                          total_size, tag_name);
        (void)total_size;

    } else if(service_code == 0x4D || service_code == 0x4E || service_code == 0x53) {
        /* Write Tag Service */
        /* Request data: data_type (UINT16) + element_count (UINT16) + data */
        EipUint16 data_type = req_data[0] | (req_data[1] << 8);
        EipUint16 element_count = req_data[2] | (req_data[3] << 8);
        if(element_count == 0) element_count = 1;

        const EipUint8 *write_data = req_data + 4;
        size_t write_size = tag->data_size * element_count;

        OPENER_TRACE_INFO("DoXOM Tag: writing %u elements of '%s' (type=0x%02X)\n",
                          element_count, tag_name, data_type);

        (void)data_type;  /* We trust the registered type */

        /* Copy data to tag storage */
        memcpy(tag->data, write_data, write_size);

        /* Build proper CIP response for Write Tag:
         *   reply_service(1B) + reserved(1B) + general_status(1B) + addl_size(1B) = 4 bytes */
        response->message.message_buffer[0] = (uint8_t)(0x80 | service_code);
        response->message.message_buffer[1] = 0x00;
        response->message.message_buffer[2] = 0x00;
        response->message.message_buffer[3] = 0x00;
        response->message.used_message_length = 4;
        response->message.current_message_position = response->message.message_buffer + 4;

        response->general_status = 0x00; /* Success */
        response->size_of_additional_status = 0;

        OPENER_TRACE_INFO("DoXOM Tag: wrote %zu bytes to '%s'\n",
                          write_size, tag_name);
    } else {
        return kEipStatusError;
    }

    return kEipStatusOkSend;
}