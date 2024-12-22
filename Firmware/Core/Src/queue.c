#include "queue.h"

/* Private function prototypes ---------------------------------------------*/
static uint16_t modulo(uint16_t value, uint16_t size);

/* Exported functions ------------------------------------------------------*/
void Queue_InitBuffer(Queue_Buffer_t *buff, qint7_8_t *container, uint16_t capacityInNumOfItems)
{
  buff->head = 0;
  buff->tail = 0;
  buff->pContainer = container;
  buff->bufferSize = QUEUE_REQUIRED_BUFFER_SIZE(capacityInNumOfItems);
}

void Queue_ClearBuffer(Queue_Buffer_t *buff)
{
  buff->head = 0;
  buff->tail = 0;
}

void Queue_Enqueue(Queue_Buffer_t *buff, qint7_8_t element)
{
  buff->pContainer[buff->tail++] = element;
  buff->tail = modulo(buff->tail, buff->bufferSize);
}

qint7_8_t Queue_Dequeue(Queue_Buffer_t *buff)
{
  qint7_8_t element;
  element = buff->pContainer[buff->head++];
  buff->head = modulo(buff->head, buff->bufferSize);

  return element;
}

void Queue_Remove(Queue_Buffer_t *buff, uint16_t elementIndex)
{
  if (elementIndex <= Queue_GetElementCount(buff))
  {
    buff->head += elementIndex;
    buff->head = modulo(buff->head, buff->bufferSize);
  }
}

uint16_t Queue_Search(Queue_Buffer_t *buff, uint8_t element)
{
  uint16_t num_of_elements;
  num_of_elements = Queue_GetElementCount(buff);

  for (uint16_t i = 0; i < num_of_elements; i++)
  {
    if (Queue_Peek(buff, i) == element)
    {
      return i;
    }
  }

  return 0xFFFF;
}

qint7_8_t Queue_Peek(Queue_Buffer_t *buff, uint16_t elementIndex)
{
  uint16_t element_position = buff->head + elementIndex;
  element_position = modulo(element_position, buff->bufferSize);

  return (buff->pContainer[element_position]);
}

Bool_t Queue_IsEmpty(Queue_Buffer_t *buff)
{
  return (Queue_GetElementCount(buff) == 0);
}

Bool_t Queue_IsFull(Queue_Buffer_t *buff)
{
  return (Queue_GetAvailableSpace(buff) == 0);
}

uint16_t Queue_GetAvailableSpace(Queue_Buffer_t *buff)
{
  uint16_t elements;

  elements = Queue_GetElementCount(buff);

  return ((buff->bufferSize - 1) - elements);
}

uint16_t Queue_GetElementCount(Queue_Buffer_t *buff)
{
  uint16_t elements;

  if (buff->tail >= buff->head)
  {
    elements = buff->tail - buff->head;
  }
  else
  {
    elements = buff->tail + buff->bufferSize - buff->head;
  }

  return elements;
}

uint16_t modulo(uint16_t value, uint16_t size)
{
  while (value >= size)
  {
    value -= size;
  }

  return value;
}