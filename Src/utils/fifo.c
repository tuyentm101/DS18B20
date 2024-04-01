/*
 *	fifo.c
 *
 *  Author: Manh Tuyen Ta
 */

/* Includes ------------------------------------------------------------------*/
#include "fifo.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint16_t elementSize;                     // size of each element in fifo
    uint16_t nElement;                        // number of elements in fifo
    uint8_t *p_internalDataBuffer;            // pointer to internal data
	uint16_t writeEnd;                        // add items to writeEnd
    uint16_t readEnd;                         // remove items from readEnd
} Fifo_t;

/* Private variables ---------------------------------------------------------*/
static Fifo_t fifos[NUMBER_OF_FIFOS];

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define IS_FIFO_INDEX_VALID(index)		(index >= 0 && index < NUMBER_OF_FIFOS)

/* Private function prototypes -----------------------------------------------*/
static inline uint16_t fifoIsFull(const Fifo_t *const p_fifo);
static inline uint16_t fifoIsEmpty(const Fifo_t *const p_fifo);

/* Private functions ---------------------------------------------------------*/
static inline uint16_t fifoIsFull(const Fifo_t *const p_fifo)
{
	return ((p_fifo->writeEnd - p_fifo->readEnd) == p_fifo->nElement) ? 1 : 0;
}

static inline uint16_t fifoIsEmpty(const Fifo_t *const p_fifo)
{
	return (p_fifo->writeEnd == p_fifo->readEnd) ? 1 : 0;
}

/* Exported functions --------------------------------------------------------*/
uint16_t fifoCurrentSize(FifoIndex_t index)
{
	return (fifos[index].writeEnd - fifos[index].readEnd);
}

FifoIndex_t fifoInit(uint16_t elementSize, uint16_t nElement, uint8_t *const p_internalDataBuffer)
{
	static int index = 0; // count number of used circular fifos

	if (index < NUMBER_OF_FIFOS) {
		if (p_internalDataBuffer && nElement > 0 && elementSize > 0) {
			/* Check that if the size of the circular fifo is a power of 2 */
			if (0 == ((nElement - 1) & nElement)) {

				/* Initialize the ring buffer internal variables */
				fifos[index].writeEnd = 0;
				fifos[index].readEnd = 0;
				fifos[index].p_internalDataBuffer = (uint8_t*)p_internalDataBuffer;
				fifos[index].elementSize = elementSize;
				fifos[index].nElement = nElement;
				return index++;
			}
		}
	}

	return -1;
}

int16_t fifoPushOne(FifoIndex_t index, const uint8_t *p_data)
{
	int16_t ret = -1;

	if (IS_FIFO_INDEX_VALID(index)
		&& !fifoIsFull(&fifos[index]))
	{
		// calculate correct position of new element in fifo by modulo division
		uint16_t offset = (fifos[index].writeEnd & (fifos[index].nElement - 1)) * fifos[index].elementSize;
		uint16_t byteIndex = 0;

		// copy data into correct position
		// memcpy(&fifos[index].p_internalDataBuffer[offset], p_data, fifos[index].elementSize);
		for (; byteIndex < fifos[index].elementSize; ++byteIndex) {
			fifos[index].p_internalDataBuffer[offset + byteIndex] = p_data[byteIndex];
		}

		++fifos[index].writeEnd;
		ret = 0;
	}

	return ret;
}

int16_t fifoPopOne(FifoIndex_t index, uint8_t *p_data)
{
	int16_t ret = -1;

	if (IS_FIFO_INDEX_VALID(index)
		&& !fifoIsEmpty(&fifos[index]))
	{
		// calculate correct position of element at readEnd to be extracted from fifo
		uint16_t offset = (fifos[index].readEnd & (fifos[index].nElement - 1)) * fifos[index].elementSize;
		uint16_t byteIndex;

		// copy from position of interest into pointer of data
		// memcpy(p_data, &fifos[index].p_internalDataBuffer[offset], fifos[index].elementSize);
		for (byteIndex = 0; byteIndex < fifos[index].elementSize; ++byteIndex) {
			p_data[byteIndex] = fifos[index].p_internalDataBuffer[offset + byteIndex];
		}

		++fifos[index].readEnd;
		ret = 0;
	}

	return ret;
}

int16_t fifoPopMultiple(FifoIndex_t index, uint16_t nElement, uint8_t *p_data)
{
	int16_t ret = -1;

	if (IS_FIFO_INDEX_VALID(index)
		&& !fifoIsEmpty(&fifos[index])
		&& nElement <= fifoCurrentSize(index))
	{
		uint16_t offset;
		uint16_t byteIndex;

		for (uint16_t i = 0; i < nElement; ++i) {
			offset = (fifos[index].readEnd & (fifos[index].nElement - 1)) * fifos[index].elementSize;

			// copy from position of interest into pointer of data
			// memcpy(p_data + i * fifos[index].elementSize, &fifos[index].p_internalDataBuffer[offset], fifos[index].elementSize);
			for (byteIndex = 0; byteIndex < fifos[index].elementSize; ++byteIndex) {
				p_data[fifos[index].elementSize * i + byteIndex] = fifos[index].p_internalDataBuffer[offset + byteIndex];
			}

			++fifos[index].readEnd;
		}
		ret = 0;
	}

	return ret;
}

int16_t fifoGetOne(FifoIndex_t index, uint8_t *const p_data)
{
	int16_t ret = -1;

	if (IS_FIFO_INDEX_VALID(index)
		&& !fifoIsEmpty(&fifos[index]))
	{
		// calculate correct position of element at readEnd to be read from fifo
		uint16_t offset = (fifos[index].readEnd & (fifos[index].nElement - 1)) * fifos[index].elementSize;
		uint16_t byteIndex = 0;

		// copy from position of interest into pointer of data
		// memcpy(p_data, &fifos[index].p_internalDataBuffer[offset], fifos[index].elementSize);
		for (byteIndex = 0; byteIndex < fifos[index].elementSize; ++byteIndex) {
			p_data[byteIndex] = fifos[index].p_internalDataBuffer[offset + byteIndex];
		}

		ret = 0;
	}

	return ret;
}

int16_t fifoGetMultiple(FifoIndex_t index, uint16_t nElement, uint8_t *const p_data)
{
	int16_t ret = -1;

	if (IS_FIFO_INDEX_VALID(index)
		&& !fifoIsEmpty(&fifos[index])
		&& nElement <= fifoCurrentSize(index))
	{
		uint16_t currentReadEnd;
		uint16_t offset;
		uint16_t byteIndex;

		for (uint16_t i = 0; i < nElement; ++i) {
			currentReadEnd = fifos[index].readEnd + i;
			offset = (currentReadEnd & (fifos[index].nElement - 1)) * fifos[index].elementSize;
			// copy from position of interest into pointer of data
			// memcpy(p_data + i * fifos[index].elementSize, &fifos[index].p_internalDataBuffer[offset], fifos[index].elementSize);
			for (byteIndex = 0; byteIndex < fifos[index].elementSize; ++byteIndex) {
				p_data[fifos[index].elementSize * i + byteIndex] = fifos[index].p_internalDataBuffer[offset + byteIndex];
			}
		}
		ret = 0;
	}
	
	return ret;
}

/*****END OF FILE****/
