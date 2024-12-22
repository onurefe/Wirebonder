/***
 * @author     Onur Efe
 */
#ifndef QUEUE_H
#define QUEUE_H

/* Includes ------------------------------------------------------------------*/
#include "generic.h"

#define QUEUE_REQUIRED_BUFFER_SIZE(NUM_OF_ITEMS) ((NUM_OF_ITEMS) + 1)

/* Typedefs ------------------------------------------------------------------*/
typedef struct
{
  uint16_t tail;
  uint16_t head;
  uint16_t bufferSize;
  qint7_8_t *pContainer;
} Queue_Buffer_t;

/* Exported functions --------------------------------------------------------*/
/***
 * @Brief      Creates a buffer, allocates it's memory and returns the pointer
 *             of it.
 * @Params     buff-> Pointer to buffer.
 *             container-> Pointer of the data container.
 *             size-> Size of the data container.
 *
 * @Return     None.
 */
extern void Queue_InitBuffer(Queue_Buffer_t *buff, qint7_8_t *container, uint16_t capacityInNumOfItems);

/***
 * @Brief      Clears the addressed buffer.
 *
 * @Params     buff-> Pointer to the buffer.
 */
extern void Queue_ClearBuffer(Queue_Buffer_t *buff);

/***
 * @Brief      Enqueues byte to the given buffer.
 *
 * @Params     buff-> Pointer to the buffer.
 *             byte-> Byte to be enqueued.
 */
extern void Queue_Enqueue(Queue_Buffer_t *buff, qint7_8_t byte);

/***
 * @Brief      Dequeues byte from the given buffer.
 *
 * @Params     buff-> Pointer to the buffer.
 *
 * @Return     Byte.
 */
extern qint7_8_t Queue_Dequeue(Queue_Buffer_t *buff);

/***
 * @Brief      Removes elements until the indexed element(also including).
 *
 * @Params     buff-> Pointer to the buffer.
 *             		elementIndex-> Index of the element.
 *
 * @Return     None.
 */
extern void Queue_Remove(Queue_Buffer_t *buff, uint16_t elementIndex);

/***
 * @Brief      Searches an element in the buffer. Returns element index if the element
 *             exists. Returns -1 otherwise.
 *
 * @Params     buff-> Buffer to be searched.
 *             	element-> Element value.
 *
 * @Return     Element index or -1.
 */
extern uint16_t Queue_Search(Queue_Buffer_t *buff, uint8_t element);

/***
 * @Brief      Peeks element in the buffer. Indexing starts at the first element.
 *
 * @Params     pBuff-> Pointer to the buffer.
 *             		elementIndex-> Index of the element.
 *
 * @Return     Element value.
 */
extern qint7_8_t Queue_Peek(Queue_Buffer_t *buff, uint16_t elementIndex);

/***
 * @Brief      Returns number of elements the buffer contains.
 *
 * @Params     buff-> Buffer pointer.
 *
 * @Return     Element count.
 */
extern uint16_t Queue_GetElementCount(Queue_Buffer_t *buff);

/***
 * @Brief      Returns available space of the buffer.
 *
 * @Params     buff-> Buffer pointer.
 *
 * @Return     Available space.
 */
extern uint16_t Queue_GetAvailableSpace(Queue_Buffer_t *buff);

/***
 * @Brief      Checks if the buffer is empty.
 *
 * @Params     buff-> Pointer to buffer.
 *
 * @Return     TRUE or FALSE.
 */
extern Bool_t Queue_IsEmpty(Queue_Buffer_t *buff);

/***
 * @Brief      Checks if the buffer is full.
 *
 * @Params     buff-> Pointer to buffer.
 *
 * @Return     TRUE or FALSE.
 */
extern Bool_t Queue_IsFull(Queue_Buffer_t *buff);

#endif