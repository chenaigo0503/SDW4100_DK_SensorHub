/*************************************************************************************************/
/*!
 *  \file   crc8.h
 *
 *  \brief  CRC-8 utilities.
 *
 *          $Date: 2016-12-28 16:12:14 -0600 (Wed, 28 Dec 2016) $
 *          $Revision: 10805 $
 *
 *
 *          $Date: 2020-9-4 10:59:50 $
 *          $Revision: v0.1 $
 *
 */
/*************************************************************************************************/
#ifndef CRC8_H
#define CRC8_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************************************/
/*!
 *  \fn     CalcCrc8
 *
 *  \brief  Calculate the CRC-8 of the given buffer or by algorithm.
 *
 *  \model  CRC-8/MAXIM
 *          Polyomial x8+x5+x4+1 0x131(0x31)
 *          crcTnit   0x00
 *          REFIN     True
 *          REFOUT    True
 *
 *  \param  pBuf     Buffer to compute the CRC.
 *  \param  len      Length of the buffer.
 *
 *  \return crc.
 */
/*************************************************************************************************/
uint8_t CalcCrc8(uint8_t *pBuf, uint32_t len);

#ifdef __cplusplus
};
#endif

#endif /* CRC8_H */
