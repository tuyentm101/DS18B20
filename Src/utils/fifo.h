/*
 * fifo.h
 *
 *  Author: Manh Tuyen Ta
 */
 
#ifndef __FIFO_H
#define	__FIFO_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h> 

/* Exported types ------------------------------------------------------------*/
typedef int16_t FifoIndex_t;

/* Exported constants --------------------------------------------------------*/
#define NUMBER_OF_FIFOS			16   // maximum number of circular fifos

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */ 

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/**
  * @brief init a new circular fifo
  * @param elementSize: size of element in internal array
  * @param nElement: number of element in internal data buffer, need to be the power of 2: 2, 4, 8, 16, ...
  * @param p_internalDataBuffer: pointer to internal data buffer
  * @return new circular fifo index, in range of 0 ... NUMBER_OF_FIFOS - 1
  * @return >= 0 if successful
  * @return -1 if failed
  */
FifoIndex_t fifoInit(uint16_t elementSize, uint16_t nElement, uint8_t *const p_internalDataBuffer);

/**
  * @brief put one data element into tail of fifo
  * @param index: index of operating fifo
  * @param p_data: pointer to data element to be put
  * @return 0 if successful
  * @return -1 if failed
  */
int16_t fifoPushOne(FifoIndex_t index, const uint8_t *p_data);

/**
  * @brief extract one data element from head of fifo
  * @param index: index of operating fifo
  * @param p_data: pointer to data element to be extracted into
  * @return 0 if successful
  * @return -1 if failed
  */
int16_t fifoPopOne(FifoIndex_t index, uint8_t *p_data);

/**
 * @brief
 * @param
 * @return 0 if successful
 * @return -1 if failed
*/
int16_t fifoPopMultiple(FifoIndex_t index, uint16_t nElement, uint8_t *p_data);

/**
 * @brief access one data element from head of fifo
 * @param index: index of operating fifo
 * @param p_data: pointer to data element to be gotten into
 * @return 0 if successful
 * @return -1 if failed
*/
int16_t fifoGetOne(FifoIndex_t index, uint8_t *const p_data);

/**
 * @brief
 * @param
 * @return 0 if successful
 * @return -1 if failed
*/
int16_t fifoGetMultiple(FifoIndex_t index, uint16_t nElement, uint8_t *const p_data);

/**
 * @brief query current number of elements in fifo
 * @param index: index of operating fifo
 * @return current number of elements in fifo
*/
uint16_t fifoCurrentSize(FifoIndex_t index);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* __FIFO_H */
