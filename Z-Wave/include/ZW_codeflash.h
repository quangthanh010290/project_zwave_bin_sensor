/****************************************************************************
 *
 * Copyright (c) 2001-2012
 * Sigma Designs, Inc.
 * All Rights Reserved
 *
 *---------------------------------------------------------------------------
 *
 * Description: ZW030x internal code FLASH interface
 *
 * Author:   Erik Friis Harck
 *
 * Last Changed By:  $Author: jsi $
 * Revision:         $Revision: 18790 $
 * Last Changed:     $Date: 2010-09-24 14:50:51 +0200 (fr, 24 sep 2010) $
 *
 ****************************************************************************/
#ifndef _ZW_CODEFLASH_H_
#define _ZW_CODEFLASH_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#define CODE_FLASH_PAGE_SIZE      256

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

extern BYTE                         /*RET Number of bytes written +1 */
ZW_CodeFlashBufferWrite(
  WORD  offset,                     /* IN Application area offset */
  WORD  length,                     /* IN Number of bytes to copy */
  BYTE  *buffer);                   /* IN Buffer pointer          */

extern void ZW_CodeFlashFlush(void);

extern void ZW_CodeFlashInit(void);

#endif /* _ZW_CODEFLASH_H_ */
