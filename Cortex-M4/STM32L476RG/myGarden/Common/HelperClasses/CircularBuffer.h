/**
 * @file CircularBuffer.h
 * @author DGM
 * \brief This file implements a container wrapper class to provide circular buffering features to a generic pre-allocated contiguous memory array.
 * \note Non-reentrant & not thread-safe
 * @version 0.1
 * @date 2021-11-07
 */

#ifndef __CIRCULAR_BUFFER_H_
#define __CIRCULAR_BUFFER_H_

template <typename T> class CircularBuffer
{
public:

	/**
	 * \brief Construct a new CircularBuffer object
	 * \param capacity Maximum number of elements the buffer can store
	 * \param buffer Pointer to pre-allocated memory array of elements
	 */
	explicit CircularBuffer(size_t capacity, T * buffer) :
		data(buffer),
		tail(0),
		head(0),
		length(0),
		capacity(capacity)
	{}

	/**
	 * \brief Check if the buffer is empty
	 * \return true if empty, false otherwise
	 */
	inline bool Empty()
	{
		return (length == 0);
	}

	/**
	 * \brief Check if the buffer is full
	 * \return true if full, false otherwise
	 */
	inline bool Full()
	{
		return (length == capacity);
	}

	/**
	 * \brief Push an element to the back of the buffer
	 * \param element Const reference of the element to be pushed
	 */
	void Push(const T& element)
	{
		if( !Full() )
		{
			*(data + tail++) = element;
			tail = (tail == capacity) ? 0 : tail;
			length++;
		}
		else
		{
			assert(0); //shouldn't happen
		}
	}

	/**
	 * \brief Pops and element from the front of the buffer
	 * \return T Element retrieved
	 */
	T Pop()
	{
		T poppedElement;
		if( !Empty() )
		{
			poppedElement = *(data + head++);
			head = (head == capacity) ? 0 : head;
			length--;
		}
		else
		{
			assert(0); //shouldn't happen
		}
		return poppedElement;
	}

	/**
	 * \brief Resets the buffer to its initial state
	 */
	void Reset()
	{
		head = tail = length = 0;
	}

private:
	T * data; /**< Internal pointer to the pre-allocated memory array */
	size_t tail; /**< Rolling index pointing to the back of the buffer */
	size_t head; /**< Rolling index pointing to the front of the buffer */
	size_t length; /**< Current number of elements stored in the buffer */
	size_t capacity; /**< Maximum number of elements the buffer can store */
};

#endif //#ifndef __CIRCULAR_BUFFER_H_