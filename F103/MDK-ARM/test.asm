*------------------------------------------------------------------------------
 * uVision/ARM development tools
 * Copyright (C) 2015-2020 ARM Ltd and ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    test.asm
 * Purpose: ROM Image generated from user specified files.
 * Note:    Generated by FCARM FILE CONVERTER V2.58, do not modify!
 *----------------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>

extern const uint32_t imageLastModified;
extern uint32_t imageFileInfo (const char *name, const uint8_t **data);

/* File information */
typedef struct _imageFileItem {
  uint32_t       id;            /* Name identifier (CRC32 value of file name) */
  const uint8_t *data;          /* Data start address in ROM                  */
} imageFileItem;

#define IMAGE_FILE_COUNT 2U

/* Last-Modified: Sun,    Nov 2024 08:02:07 GMT */
const uint32_t imageLastModified = 1730620927U;

static const uint8_t imageFileData[2707U] = {

  /*-- File: ..\DHT11\dht11.c, 2367 bytes --*/
  0x23U,0x69U,0x6EU,0x63U,0x6CU,0x75U,0x64U,0x65U,0x20U,0x22U,0x73U,0x74U,0x6DU,
  0x33U,0x32U,0x66U,0x31U,0x78U,0x78U,0x5FU,0x68U,0x61U,0x6CU,0x2EU,0x68U,0x22U,
  0x0DU,0x0AU,0x23U,0x69U,0x6EU,0x63U,0x6CU,0x75U,0x64U,0x65U,0x20U,0x22U,0x6DU,
  0x61U,0x69U,0x6EU,0x2EU,0x68U,0x22U,0x0DU,0x0AU,0x23U,0x69U,0x6EU,0x63U,0x6CU,
  0x75U,0x64U,0x65U,0x20U,0x22U,0x64U,0x68U,0x74U,0x31U,0x31U,0x2EU,0x68U,0x22U,
  0x0DU,0x0AU,0x23U,0x64U,0x65U,0x66U,0x69U,0x6EU,0x65U,0x20U,0x44U,0x48U,0x54U,
  0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,
  0x49U,0x4EU,0x5FU,0x37U,0x20U,0x20U,0x0DU,0x0AU,0x23U,0x64U,0x65U,0x66U,0x69U,
  0x6EU,0x65U,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,
  0x20U,0x47U,0x50U,0x49U,0x4FU,0x41U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x0DU,
  0x0AU,0x0DU,0x0AU,0x76U,0x6FU,0x6CU,0x61U,0x74U,0x69U,0x6CU,0x65U,0x20U,0x75U,
  0x69U,0x6EU,0x74U,0x33U,0x32U,0x5FU,0x74U,0x20U,0x64U,0x65U,0x6CU,0x61U,0x79U,
  0x5FU,0x75U,0x73U,0x20U,0x3DU,0x20U,0x30U,0x3BU,0x0DU,0x0AU,0x0DU,0x0AU,0x2FU,
  0x2FU,0x20U,0x48U,0xE0U,0x6DU,0x20U,0x73U,0x65U,0x74U,0x20U,0x63U,0x68U,0xE2U,
  0x6EU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x20U,0x6CU,0xE0U,0x6DU,0x20U,0x4FU,0x75U,
  0x74U,0x70U,0x75U,0x74U,0x0DU,0x0AU,0x76U,0x6FU,0x69U,0x64U,0x20U,0x44U,0x48U,
  0x54U,0x31U,0x31U,0x5FU,0x53U,0x65U,0x74U,0x5FU,0x50U,0x69U,0x6EU,0x5FU,0x4FU,
  0x75U,0x74U,0x70U,0x75U,0x74U,0x28U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x54U,0x79U,
  0x70U,0x65U,0x44U,0x65U,0x66U,0x20U,0x2AU,0x47U,0x50U,0x49U,0x4FU,0x78U,0x2CU,
  0x20U,0x75U,0x69U,0x6EU,0x74U,0x31U,0x36U,0x5FU,0x74U,0x20U,0x47U,0x50U,0x49U,
  0x4FU,0x5FU,0x50U,0x69U,0x6EU,0x29U,0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,
  0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,0x54U,0x79U,0x70U,
  0x65U,0x44U,0x65U,0x66U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,
  0x74U,0x53U,0x74U,0x72U,0x75U,0x63U,0x74U,0x20U,0x3DU,0x20U,0x7BU,0x30U,0x7DU,
  0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,
  0x6EU,0x69U,0x74U,0x53U,0x74U,0x72U,0x75U,0x63U,0x74U,0x2EU,0x50U,0x69U,0x6EU,
  0x20U,0x3DU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x69U,0x6EU,0x3BU,0x0DU,
  0x0AU,0x20U,0x20U,0x20U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,
  0x74U,0x53U,0x74U,0x72U,0x75U,0x63U,0x74U,0x2EU,0x4DU,0x6FU,0x64U,0x65U,0x20U,
  0x3DU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x4DU,0x4FU,0x44U,0x45U,0x5FU,0x4FU,
  0x55U,0x54U,0x50U,0x55U,0x54U,0x5FU,0x50U,0x50U,0x3BU,0x0DU,0x0AU,0x20U,0x20U,
  0x20U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,0x53U,0x74U,
  0x72U,0x75U,0x63U,0x74U,0x2EU,0x53U,0x70U,0x65U,0x65U,0x64U,0x20U,0x3DU,0x20U,
  0x47U,0x50U,0x49U,0x4FU,0x5FU,0x53U,0x50U,0x45U,0x45U,0x44U,0x5FU,0x46U,0x52U,
  0x45U,0x51U,0x5FU,0x4CU,0x4FU,0x57U,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,
  0x48U,0x41U,0x4CU,0x5FU,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,
  0x28U,0x47U,0x50U,0x49U,0x4FU,0x78U,0x2CU,0x20U,0x26U,0x47U,0x50U,0x49U,0x4FU,
  0x5FU,0x49U,0x6EU,0x69U,0x74U,0x53U,0x74U,0x72U,0x75U,0x63U,0x74U,0x29U,0x3BU,
  0x0DU,0x0AU,0x7DU,0x0DU,0x0AU,0x0DU,0x0AU,0x2FU,0x2FU,0x20U,0x48U,0xE0U,0x6DU,
  0x20U,0x73U,0x65U,0x74U,0x20U,0x63U,0x68U,0xE2U,0x6EU,0x20U,0x47U,0x50U,0x49U,
  0x4FU,0x20U,0x6CU,0xE0U,0x6DU,0x20U,0x49U,0x6EU,0x70U,0x75U,0x74U,0x0DU,0x0AU,
  0x76U,0x6FU,0x69U,0x64U,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x53U,0x65U,
  0x74U,0x5FU,0x50U,0x69U,0x6EU,0x5FU,0x49U,0x6EU,0x70U,0x75U,0x74U,0x28U,0x47U,
  0x50U,0x49U,0x4FU,0x5FU,0x54U,0x79U,0x70U,0x65U,0x44U,0x65U,0x66U,0x20U,0x2AU,
  0x47U,0x50U,0x49U,0x4FU,0x78U,0x2CU,0x20U,0x75U,0x69U,0x6EU,0x74U,0x31U,0x36U,
  0x5FU,0x74U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x69U,0x6EU,0x29U,0x20U,
  0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,
  0x6EU,0x69U,0x74U,0x54U,0x79U,0x70U,0x65U,0x44U,0x65U,0x66U,0x20U,0x47U,0x50U,
  0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,0x53U,0x74U,0x72U,0x75U,0x63U,0x74U,
  0x20U,0x3DU,0x20U,0x7BU,0x30U,0x7DU,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,
  0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,0x53U,0x74U,0x72U,0x75U,
  0x63U,0x74U,0x2EU,0x50U,0x69U,0x6EU,0x20U,0x3DU,0x20U,0x47U,0x50U,0x49U,0x4FU,
  0x5FU,0x50U,0x69U,0x6EU,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x47U,0x50U,
  0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,0x53U,0x74U,0x72U,0x75U,0x63U,0x74U,
  0x2EU,0x4DU,0x6FU,0x64U,0x65U,0x20U,0x3DU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,
  0x4DU,0x4FU,0x44U,0x45U,0x5FU,0x49U,0x4EU,0x50U,0x55U,0x54U,0x3BU,0x0DU,0x0AU,
  0x20U,0x20U,0x20U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,
  0x53U,0x74U,0x72U,0x75U,0x63U,0x74U,0x2EU,0x50U,0x75U,0x6CU,0x6CU,0x20U,0x3DU,
  0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x4EU,0x4FU,0x50U,0x55U,0x4CU,0x4CU,0x3BU,
  0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x48U,0x41U,0x4CU,0x5FU,0x47U,0x50U,0x49U,
  0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,0x28U,0x47U,0x50U,0x49U,0x4FU,0x78U,0x2CU,
  0x20U,0x26U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x49U,0x6EU,0x69U,0x74U,0x53U,0x74U,
  0x72U,0x75U,0x63U,0x74U,0x29U,0x3BU,0x0DU,0x0AU,0x7DU,0x0DU,0x0AU,0x0DU,0x0AU,
  0x76U,0x6FU,0x69U,0x64U,0x20U,0x44U,0x65U,0x6CU,0x61U,0x79U,0x5FU,0x75U,0x73U,
  0x28U,0x75U,0x69U,0x6EU,0x74U,0x33U,0x32U,0x5FU,0x74U,0x20U,0x75U,0x73U,0x29U,
  0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x64U,0x65U,0x6CU,0x61U,0x79U,
  0x5FU,0x75U,0x73U,0x20U,0x3DU,0x20U,0x75U,0x73U,0x3BU,0x0DU,0x0AU,0x20U,0x20U,
  0x20U,0x20U,0x77U,0x68U,0x69U,0x6CU,0x65U,0x20U,0x28U,0x64U,0x65U,0x6CU,0x61U,
  0x79U,0x5FU,0x75U,0x73U,0x20U,0x3EU,0x20U,0x30U,0x29U,0x3BU,0x0DU,0x0AU,0x7DU,
  0x0DU,0x0AU,0x0DU,0x0AU,0x0DU,0x0AU,0x76U,0x6FU,0x69U,0x64U,0x20U,0x48U,0x41U,
  0x4CU,0x5FU,0x54U,0x49U,0x4DU,0x5FU,0x50U,0x65U,0x72U,0x69U,0x6FU,0x64U,0x45U,
  0x6CU,0x61U,0x70U,0x73U,0x65U,0x64U,0x43U,0x61U,0x6CU,0x6CU,0x62U,0x61U,0x63U,
  0x6BU,0x28U,0x54U,0x49U,0x4DU,0x5FU,0x48U,0x61U,0x6EU,0x64U,0x6CU,0x65U,0x54U,
  0x79U,0x70U,0x65U,0x44U,0x65U,0x66U,0x20U,0x2AU,0x68U,0x74U,0x69U,0x6DU,0x29U,
  0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x69U,0x66U,0x20U,0x28U,0x64U,
  0x65U,0x6CU,0x61U,0x79U,0x5FU,0x75U,0x73U,0x20U,0x3EU,0x20U,0x30U,0x29U,0x20U,
  0x64U,0x65U,0x6CU,0x61U,0x79U,0x5FU,0x75U,0x73U,0x2DU,0x2DU,0x3BU,0x0DU,0x0AU,
  0x7DU,0x0DU,0x0AU,0x0DU,0x0AU,0x2FU,0x2FU,0x47U,0x69U,0x61U,0x69U,0x20U,0x6DU,
  0x61U,0x78U,0x20U,0x74U,0x75U,0x20U,0x68U,0x65U,0x78U,0x20U,0x73U,0x61U,0x6EU,
  0x67U,0x20U,0x74U,0x68U,0x61U,0x70U,0x20U,0x70U,0x68U,0x61U,0x6EU,0x0DU,0x0AU,
  0x75U,0x69U,0x6EU,0x74U,0x38U,0x5FU,0x74U,0x20U,0x44U,0x65U,0x63U,0x6FU,0x64U,
  0x65U,0x28U,0x75U,0x69U,0x6EU,0x74U,0x38U,0x5FU,0x74U,0x20U,0x74U,0x65U,0x6DU,
  0x70U,0x29U,0x7BU,0x0DU,0x0AU,0x09U,0x72U,0x65U,0x74U,0x75U,0x72U,0x6EU,0x20U,
  0x28U,0x28U,0x28U,0x74U,0x65U,0x6DU,0x70U,0x20U,0x26U,0x20U,0x30U,0x78U,0x66U,
  0x30U,0x29U,0x20U,0x3EU,0x3EU,0x20U,0x34U,0x29U,0x20U,0x2AU,0x20U,0x31U,0x30U,
  0x29U,0x20U,0x2BU,0x20U,0x28U,0x74U,0x65U,0x6DU,0x70U,0x20U,0x26U,0x20U,0x30U,
  0x78U,0x30U,0x66U,0x29U,0x3BU,0x0DU,0x0AU,0x7DU,0x0DU,0x0AU,0x0DU,0x0AU,0x0DU,
  0x0AU,0x75U,0x69U,0x6EU,0x74U,0x38U,0x5FU,0x74U,0x20U,0x44U,0x48U,0x54U,0x31U,
  0x31U,0x5FU,0x52U,0x65U,0x61U,0x64U,0x5FU,0x44U,0x61U,0x74U,0x61U,0x28U,0x75U,
  0x69U,0x6EU,0x74U,0x38U,0x5FU,0x74U,0x20U,0x2AU,0x74U,0x65U,0x6DU,0x70U,0x65U,
  0x72U,0x61U,0x74U,0x75U,0x72U,0x65U,0x2CU,0x20U,0x75U,0x69U,0x6EU,0x74U,0x38U,
  0x5FU,0x74U,0x20U,0x2AU,0x68U,0x75U,0x6DU,0x69U,0x64U,0x69U,0x74U,0x79U,0x29U,
  0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x75U,0x69U,0x6EU,0x74U,0x38U,
  0x5FU,0x74U,0x20U,0x62U,0x69U,0x74U,0x73U,0x5BU,0x35U,0x5DU,0x20U,0x3DU,0x20U,
  0x7BU,0x30U,0x7DU,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x75U,0x69U,0x6EU,
  0x74U,0x38U,0x5FU,0x74U,0x20U,0x69U,0x2CU,0x20U,0x6AU,0x3BU,0x0DU,0x0AU,0x0DU,
  0x0AU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,
  0x53U,0x65U,0x74U,0x5FU,0x50U,0x69U,0x6EU,0x5FU,0x4FU,0x75U,0x74U,0x70U,0x75U,
  0x74U,0x28U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,
  0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x29U,0x3BU,0x20U,
  0x20U,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x48U,0x41U,0x4CU,0x5FU,0x47U,0x50U,
  0x49U,0x4FU,0x5FU,0x57U,0x72U,0x69U,0x74U,0x65U,0x50U,0x69U,0x6EU,0x28U,0x44U,
  0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,0x20U,0x44U,0x48U,
  0x54U,0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x2CU,0x20U,0x47U,0x50U,0x49U,0x4FU,
  0x5FU,0x50U,0x49U,0x4EU,0x5FU,0x52U,0x45U,0x53U,0x45U,0x54U,0x29U,0x3BU,0x0DU,
  0x0AU,0x20U,0x20U,0x20U,0x20U,0x44U,0x65U,0x6CU,0x61U,0x79U,0x5FU,0x75U,0x73U,
  0x28U,0x32U,0x30U,0x30U,0x30U,0x30U,0x29U,0x3BU,0x20U,0x0DU,0x0AU,0x0DU,0x0AU,
  0x20U,0x20U,0x20U,0x20U,0x48U,0x41U,0x4CU,0x5FU,0x47U,0x50U,0x49U,0x4FU,0x5FU,
  0x57U,0x72U,0x69U,0x74U,0x65U,0x50U,0x69U,0x6EU,0x28U,0x44U,0x48U,0x54U,0x31U,
  0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,
  0x5FU,0x50U,0x49U,0x4EU,0x2CU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x49U,
  0x4EU,0x5FU,0x53U,0x45U,0x54U,0x29U,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,
  0x44U,0x65U,0x6CU,0x61U,0x79U,0x5FU,0x75U,0x73U,0x28U,0x32U,0x30U,0x29U,0x3BU,
  0x20U,0x20U,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,
  0x5FU,0x53U,0x65U,0x74U,0x5FU,0x50U,0x69U,0x6EU,0x5FU,0x49U,0x6EU,0x70U,0x75U,
  0x74U,0x28U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,
  0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x29U,0x3BU,0x0DU,
  0x0AU,0x0DU,0x0AU,0x20U,0x20U,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x77U,0x68U,
  0x69U,0x6CU,0x65U,0x20U,0x28U,0x48U,0x41U,0x4CU,0x5FU,0x47U,0x50U,0x49U,0x4FU,
  0x5FU,0x52U,0x65U,0x61U,0x64U,0x50U,0x69U,0x6EU,0x28U,0x44U,0x48U,0x54U,0x31U,
  0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,
  0x5FU,0x50U,0x49U,0x4EU,0x29U,0x20U,0x3DU,0x3DU,0x20U,0x47U,0x50U,0x49U,0x4FU,
  0x5FU,0x50U,0x49U,0x4EU,0x5FU,0x53U,0x45U,0x54U,0x29U,0x3BU,0x0DU,0x0AU,0x20U,
  0x20U,0x20U,0x20U,0x77U,0x68U,0x69U,0x6CU,0x65U,0x20U,0x28U,0x48U,0x41U,0x4CU,
  0x5FU,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x52U,0x65U,0x61U,0x64U,0x50U,0x69U,0x6EU,
  0x28U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,0x20U,
  0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x29U,0x20U,0x3DU,0x3DU,
  0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x49U,0x4EU,0x5FU,0x52U,0x45U,0x53U,
  0x45U,0x54U,0x29U,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x77U,0x68U,0x69U,
  0x6CU,0x65U,0x20U,0x28U,0x48U,0x41U,0x4CU,0x5FU,0x47U,0x50U,0x49U,0x4FU,0x5FU,
  0x52U,0x65U,0x61U,0x64U,0x50U,0x69U,0x6EU,0x28U,0x44U,0x48U,0x54U,0x31U,0x31U,
  0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,
  0x50U,0x49U,0x4EU,0x29U,0x20U,0x3DU,0x3DU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,
  0x50U,0x49U,0x4EU,0x5FU,0x53U,0x45U,0x54U,0x29U,0x3BU,0x0DU,0x0AU,0x0DU,0x0AU,
  0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x66U,0x6FU,0x72U,0x20U,0x28U,0x6AU,0x20U,
  0x3DU,0x20U,0x30U,0x3BU,0x20U,0x6AU,0x20U,0x3CU,0x20U,0x35U,0x3BU,0x20U,0x6AU,
  0x2BU,0x2BU,0x29U,0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,
  0x20U,0x20U,0x66U,0x6FU,0x72U,0x20U,0x28U,0x69U,0x20U,0x3DU,0x20U,0x30U,0x3BU,
  0x20U,0x69U,0x20U,0x3CU,0x20U,0x38U,0x3BU,0x20U,0x69U,0x2BU,0x2BU,0x29U,0x20U,
  0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,
  0x20U,0x20U,0x77U,0x68U,0x69U,0x6CU,0x65U,0x20U,0x28U,0x48U,0x41U,0x4CU,0x5FU,
  0x47U,0x50U,0x49U,0x4FU,0x5FU,0x52U,0x65U,0x61U,0x64U,0x50U,0x69U,0x6EU,0x28U,
  0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,0x20U,0x44U,
  0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x29U,0x20U,0x3DU,0x3DU,0x20U,
  0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x49U,0x4EU,0x5FU,0x52U,0x45U,0x53U,0x45U,
  0x54U,0x29U,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,
  0x20U,0x20U,0x20U,0x20U,0x44U,0x65U,0x6CU,0x61U,0x79U,0x5FU,0x75U,0x73U,0x28U,
  0x34U,0x30U,0x29U,0x3BU,0x20U,0x20U,0x0DU,0x0AU,0x0DU,0x0AU,0x20U,0x20U,0x20U,
  0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x69U,0x66U,0x20U,0x28U,
  0x48U,0x41U,0x4CU,0x5FU,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x52U,0x65U,0x61U,0x64U,
  0x50U,0x69U,0x6EU,0x28U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,
  0x54U,0x2CU,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x29U,
  0x20U,0x3DU,0x3DU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x49U,0x4EU,0x5FU,
  0x53U,0x45U,0x54U,0x29U,0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x20U,
  0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x62U,0x69U,
  0x74U,0x73U,0x5BU,0x6AU,0x5DU,0x20U,0x7CU,0x3DU,0x20U,0x28U,0x31U,0x20U,0x3CU,
  0x3CU,0x20U,0x28U,0x37U,0x20U,0x2DU,0x20U,0x69U,0x29U,0x29U,0x3BU,0x0DU,0x0AU,
  0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x7DU,
  0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,
  0x20U,0x77U,0x68U,0x69U,0x6CU,0x65U,0x20U,0x28U,0x48U,0x41U,0x4CU,0x5FU,0x47U,
  0x50U,0x49U,0x4FU,0x5FU,0x52U,0x65U,0x61U,0x64U,0x50U,0x69U,0x6EU,0x28U,0x44U,
  0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x2CU,0x20U,0x44U,0x48U,
  0x54U,0x31U,0x31U,0x5FU,0x50U,0x49U,0x4EU,0x29U,0x20U,0x3DU,0x3DU,0x20U,0x47U,
  0x50U,0x49U,0x4FU,0x5FU,0x50U,0x49U,0x4EU,0x5FU,0x53U,0x45U,0x54U,0x29U,0x3BU,
  0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x7DU,0x0DU,0x0AU,
  0x20U,0x20U,0x20U,0x20U,0x7DU,0x0DU,0x0AU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,
  0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x69U,0x66U,0x20U,0x28U,0x62U,0x69U,0x74U,
  0x73U,0x5BU,0x30U,0x5DU,0x20U,0x2BU,0x20U,0x62U,0x69U,0x74U,0x73U,0x5BU,0x31U,
  0x5DU,0x20U,0x2BU,0x20U,0x62U,0x69U,0x74U,0x73U,0x5BU,0x32U,0x5DU,0x20U,0x2BU,
  0x20U,0x62U,0x69U,0x74U,0x73U,0x5BU,0x33U,0x5DU,0x20U,0x3DU,0x3DU,0x20U,0x62U,
  0x69U,0x74U,0x73U,0x5BU,0x34U,0x5DU,0x29U,0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,
  0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x2AU,0x68U,0x75U,0x6DU,0x69U,0x64U,0x69U,
  0x74U,0x79U,0x20U,0x3DU,0x20U,0x44U,0x65U,0x63U,0x6FU,0x64U,0x65U,0x28U,0x62U,
  0x69U,0x74U,0x73U,0x5BU,0x30U,0x5DU,0x29U,0x3BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,
  0x20U,0x20U,0x20U,0x20U,0x20U,0x2AU,0x74U,0x65U,0x6DU,0x70U,0x65U,0x72U,0x61U,
  0x74U,0x75U,0x72U,0x65U,0x20U,0x3DU,0x20U,0x20U,0x44U,0x65U,0x63U,0x6FU,0x64U,
  0x65U,0x28U,0x62U,0x69U,0x74U,0x73U,0x5BU,0x32U,0x5DU,0x29U,0x3BU,0x0DU,0x0AU,
  0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x20U,0x72U,0x65U,0x74U,0x75U,0x72U,
  0x6EU,0x20U,0x30U,0x3BU,0x20U,0x20U,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x7DU,
  0x20U,0x65U,0x6CU,0x73U,0x65U,0x20U,0x7BU,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,
  0x20U,0x20U,0x20U,0x20U,0x72U,0x65U,0x74U,0x75U,0x72U,0x6EU,0x20U,0x31U,0x3BU,
  0x20U,0x20U,0x0DU,0x0AU,0x20U,0x20U,0x20U,0x20U,0x7DU,0x0DU,0x0AU,0x7DU,0x0DU,
  0x0AU,

  /*-- File: ..\DHT11\dht11.h, 340 bytes --*/
  0x23U,0x69U,0x66U,0x6EU,0x64U,0x65U,0x66U,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,
  0x5FU,0x48U,0x0DU,0x0AU,0x23U,0x64U,0x65U,0x66U,0x69U,0x6EU,0x65U,0x20U,0x44U,
  0x48U,0x54U,0x31U,0x31U,0x5FU,0x48U,0x0DU,0x0AU,0x0DU,0x0AU,0x23U,0x69U,0x6EU,
  0x63U,0x6CU,0x75U,0x64U,0x65U,0x20U,0x22U,0x73U,0x74U,0x6DU,0x33U,0x32U,0x66U,
  0x31U,0x78U,0x78U,0x5FU,0x68U,0x61U,0x6CU,0x2EU,0x68U,0x22U,0x0DU,0x0AU,0x0DU,
  0x0AU,0x23U,0x64U,0x65U,0x66U,0x69U,0x6EU,0x65U,0x20U,0x44U,0x48U,0x54U,0x31U,
  0x31U,0x5FU,0x50U,0x49U,0x4EU,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x49U,
  0x4EU,0x5FU,0x37U,0x0DU,0x0AU,0x23U,0x64U,0x65U,0x66U,0x69U,0x6EU,0x65U,0x20U,
  0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x50U,0x4FU,0x52U,0x54U,0x20U,0x47U,0x50U,
  0x49U,0x4FU,0x41U,0x0DU,0x0AU,0x0DU,0x0AU,0x76U,0x6FU,0x69U,0x64U,0x20U,0x44U,
  0x48U,0x54U,0x31U,0x31U,0x5FU,0x53U,0x65U,0x74U,0x5FU,0x50U,0x69U,0x6EU,0x5FU,
  0x4FU,0x75U,0x74U,0x70U,0x75U,0x74U,0x28U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x54U,
  0x79U,0x70U,0x65U,0x44U,0x65U,0x66U,0x20U,0x2AU,0x47U,0x50U,0x49U,0x4FU,0x78U,
  0x2CU,0x20U,0x75U,0x69U,0x6EU,0x74U,0x31U,0x36U,0x5FU,0x74U,0x20U,0x47U,0x50U,
  0x49U,0x4FU,0x5FU,0x50U,0x69U,0x6EU,0x29U,0x3BU,0x0DU,0x0AU,0x0DU,0x0AU,0x76U,
  0x6FU,0x69U,0x64U,0x20U,0x44U,0x48U,0x54U,0x31U,0x31U,0x5FU,0x53U,0x65U,0x74U,
  0x5FU,0x50U,0x69U,0x6EU,0x5FU,0x49U,0x6EU,0x70U,0x75U,0x74U,0x28U,0x47U,0x50U,
  0x49U,0x4FU,0x5FU,0x54U,0x79U,0x70U,0x65U,0x44U,0x65U,0x66U,0x20U,0x2AU,0x47U,
  0x50U,0x49U,0x4FU,0x78U,0x2CU,0x20U,0x75U,0x69U,0x6EU,0x74U,0x31U,0x36U,0x5FU,
  0x74U,0x20U,0x47U,0x50U,0x49U,0x4FU,0x5FU,0x50U,0x69U,0x6EU,0x29U,0x3BU,0x0DU,
  0x0AU,0x0DU,0x0AU,0x75U,0x69U,0x6EU,0x74U,0x38U,0x5FU,0x74U,0x20U,0x44U,0x48U,
  0x54U,0x31U,0x31U,0x5FU,0x52U,0x65U,0x61U,0x64U,0x5FU,0x44U,0x61U,0x74U,0x61U,
  0x28U,0x75U,0x69U,0x6EU,0x74U,0x38U,0x5FU,0x74U,0x20U,0x2AU,0x74U,0x65U,0x6DU,
  0x70U,0x65U,0x72U,0x61U,0x74U,0x75U,0x72U,0x65U,0x2CU,0x20U,0x75U,0x69U,0x6EU,
  0x74U,0x38U,0x5FU,0x74U,0x20U,0x2AU,0x68U,0x75U,0x6DU,0x69U,0x64U,0x69U,0x74U,
  0x79U,0x29U,0x3BU,0x0DU,0x0AU,0x0DU,0x0AU,0x23U,0x65U,0x6EU,0x64U,0x69U,0x66U,
  0x0DU,0x0AU
};

static const imageFileItem imageFileTable[2U+1U] = {
  { 0x1F8CF4A3U, &imageFileData[0U]    },    // "../DHT11/dht11.c"
  { 0x34C73FC2U, &imageFileData[2367U] },    // "../DHT11/dht11.h"
  { 0x00000000U, &imageFileData[2707U] }
};

/*
 * Calculate 32-bit CRC (Polynom: 0x04C11DB7)
 *   Parameters:
 *     crc32:       CRC initial value
 *     val:         Input value
 *   Return value:  Calculated CRC value
 */
static uint32_t crc32_8bit (uint32_t crc32, uint8_t val) {
  uint32_t n;

  crc32 ^= ((uint32_t)val) << 24U;
  for (n = 8U; n; n--) {
    if (crc32 & 0x80000000U) {
      crc32 <<= 1U;
      crc32  ^= 0x04C11DB7U;
    } else {
      crc32 <<= 1U;
    }
  }
  return (crc32);
}

/*
 * Get file information from ROM image
 *   Parameters:
 *     name:        File name
 *     data:        Pointer where file data pointer will be written
 *   Return value:  File size
 */
uint32_t imageFileInfo (const char *name, const uint8_t **data) {
  uint32_t id, n;

  if ((name == NULL) || (data == NULL)) return 0U;

  id = 0xFFFFFFFFU;
  for (; *name; name++) {
    id = crc32_8bit(id, *name);
  }

  for (n = 0U; n < IMAGE_FILE_COUNT; n++) {
    if (imageFileTable[n].id == id) {
      *data = imageFileTable[n].data;
      return ((uint32_t)(imageFileTable[n+1].data - imageFileTable[n].data));
    }
  }
  return 0U;
}