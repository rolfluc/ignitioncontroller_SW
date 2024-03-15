/**
 * @file     version.h
 *
 * @brief    Definition of version scheme.
 *
 * @version  v1.0
 * @date     2020-05-20
 *
 * @note
 */

/* ============================================================================
** Copyright (C) 2020 Infineon. All rights reserved.
**               Infineon Technologies, PSS ACDC / DES ACDC
** ============================================================================
**
** ============================================================================
** This document contains proprietary information. Passing on and
** copying of this document, and communication of its contents is not
** permitted without prior written authorisation.
** ============================================================================
*
*/
/*lint -save -e960 */

#ifndef _VERSION_H_
#define _VERSION_H_

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdint.h>            /* uint*_t */

/** @addtogroup Infineon
 * @{
 */

/** @addtogroup Smack_sl
 * @{
 */


/** @addtogroup version
 * @{
 */

/**
 * @defgroup group_version_interface Interface of the version IDs
 * @ingroup group_interface_description
 *
 * For details about the version scheme refer to @ref pgSmack_Id.
 */

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

/**
 * @ingroup group_version_fw
 * Definition of the platform ID
 *
 * @note Each definition must not use more than 4 bit
 */
/*lint -e749 */
typedef enum
{
    DP2       = 0x2, //!< 0x2 denotes DP2 platform
    DP3       = 0x3, //!< 0x3 denotes DP3 platform
    DANDELION = 0x4  //!< 0x4 denotes Dandelion platform
} Platform_t;

/**
 * @ingroup group_version_fw
 * Definition of the platform extension ID
 *
 * @note Each definition must not use more than 4 bit
 */
typedef enum
{
    DPxA  = 0xA,   //!< 0xA denotes DPxA platform extension
    DPxB  = 0xB,   //!< 0xB denotes DPxB platform extension
    SMACK = 0xC    //!< 0xC denotes Smack platform extension
} Platform_Version_t;

/**
 * @ingroup group_version_fw
 * Definition of major, minor and step ID
 *
 * @{
 */
#define FW_VERSION_MAJOR  (0x1)         //!< Major version. Must start with 1 for tape out relevant releases.
#define FW_VERSION_MINOR  (0x1)         //!< Minor version. Must start with 0, when major version is increased.
#define FW_VERSION_STEP   (0x0000)      //!< Step version 16 bits. For internal releases.
/// @}


/*
==============================================================================
   3. TYPES
==============================================================================
*/
/**
 * @defgroup group_version_fw Type definition of FW version
 * @ingroup group_version_interface
 * @brief Type definitions for FW version scheme
 *
 * @{
 */
typedef struct
{
    uint8_t  platform;      //!< combines platform ID and platform ID extension
    uint8_t  version;       //!< combines ::FW_VERSION_MAJOR and ::FW_VERSION_MINOR
    uint16_t step;          //!< contains ::FW_VERSION_STEP
    uint8_t  commit_id[3];  //!< contains git first 3 bytes (6 hex digits) of git commit_id
    uint8_t  dirty;         //!< contains git dirty repo status
    uint32_t crc;           //!< contains code image crc
} Version_t;
/// @}

/*
==============================================================================
   4. EXPORTED DATA
==============================================================================
*/
extern const Version_t version;


/*
==============================================================================
   5. FUNCTION PROTOTYPES
==============================================================================
*/


/*
==============================================================================
   6. INLINE FUNCTIONS
==============================================================================
*/

/** @} */ /* End of group version */


/** @} */ /* End of group Smack_sl */

/** @} */ /* End of group Infineon */

#endif /* _VERSION_H_ */
