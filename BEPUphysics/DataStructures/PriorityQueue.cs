//using System;
//using System.Collections;
//using System.Collections.Generic;

//namespace BEPUphysics.DataStructures
//{
//    /// <summary>
//    /// Container allowing O(log(n)) insertion, O(1) removal of highest priority item, 
//    /// peeking, element indexing, and enumeration of objects using an array-based minheap.
//    /// Uses the IComparable interface to determine relative priority.
//    /// </summary>
//    /// <typeparam name="T">Type of object that the heap contains.</typeparam>
//    public class PriorityQueue<T> : IEnumerable<T> where T : IComparable<T>
//    {
//        internal T[] heapArray;

//        internal int count; //Synonymous with 'first empty index'

//        /// <summary>
//        /// Constructs a new priority queue.
//        /// </summary>
//        /// <param name="initialHeapSize">The size of the heap to use initially.</param>
//        public PriorityQueue(int initialHeapSize)
//        {
//            heapArray = new T[initialHeapSize];
//        }

//        /// <summary>
//        /// Constructs a new priority queue.
//        /// </summary>
//        public PriorityQueue()
//        {
//            heapArray = new T[8];
//        }

//        /// <summary>
//        /// Number of elements within the priority queue.
//        /// </summary>
//        public int Count
//        {
//            get { return count; }
//        }

//        /// <summary>
//        /// Gets an item from the heap by indexing into the internal heap array.
//        /// </summary>
//        /// <param name="i">Index used to find an element in the internal heap array.</param>
//        /// <returns>Element at the given idnex.</returns>
//        /// <exception cref="ArgumentOutOfRangeException">Thrown when no element exists at the given index.</exception>
//        public T this[int i]
//        {
//            get
//            {
//                if (i < count)
//                    return heapArray[i];
//                throw new ArgumentOutOfRangeException("i", "No element exists at the given index.");
//            }
//        }


//        /// <summary>
//        /// Pops off the root of the Priority Queue and returns it.
//        /// </summary>
//        /// <returns>Highest priority item in the queue.</returns>
//        /// <exception cref="InvalidOperationException">Thrown when they queue is empty.</exception>
//        public T Dequeue()
//        {
//            if (count == 0)
//                throw new InvalidOperationException("The queue is empty.");
//            T root = heapArray[0];
//            count--;
//            heapArray[0] = heapArray[count];
//            heapArray[count] = default(T);
//            int childA = 1;
//            int childB = 2;
//            int childToSwap = 0;
//            int current = 0;
//            while (true)
//            {
//                if (childA < count)
//                {
//                    if (childB < count)
//                        childToSwap = heapArray[childA].CompareTo(heapArray[childB]) == 1 ? childB : childA;
//                }
//                else if (childB < count)
//                    childToSwap = childB;
//                else
//                    break; //No more children.

//                T temp = heapArray[childToSwap];

//                heapArray[childToSwap] = heapArray[current];
//                heapArray[current] = temp;
//                current = childToSwap;
//            }


//            return root;
//        }

//        /// <summary>
//        /// Puts the item into the proper location within the Priority Queue.
//        /// </summary>
//        /// <param name="item">Item to insert.</param>
//        public void Enqueue(T item)
//        {
//            int heapCapacity = heapArray.Length;
//            //If it's hitting the max, then expand the array.
//            if (count == heapCapacity)
//            {
//                var tempHeap = new T[heapCapacity * 2];
//                Array.Copy(heapArray, 0, tempHeap, 0, count);
//                heapArray = tempHeap;
//            }
//            heapArray[count] = item;

//            int current = count;
//            int parent = (current - 1) / 2;
//            while (current != 0 && //If current is 0, it's at the root.
//                   heapArray[current].CompareTo(heapArray[parent]) == 1) // current ">" parent
//            {
//                //Flip the child with parent.
//                T temp = heapArray[parent];
//                heapArray[parent] = heapArray[current];
//                heapArray[current] = temp;
//                current = parent;
//                parent = (current - 1) / 2;
//            }
//            count++;
//        }

//        /// <summary>
//        /// Returns the highest priority item without removing it from the queue.
//        /// </summary>
//        /// <returns>Highest priority item in the queue.</returns>
//        /// <exception cref="InvalidOperationException">Thrown when the queue is empty.</exception>
//        public T Peek()
//        {
//            if (count == 0)
//                throw new InvalidOperationException("The queue is empty.");
//            return heapArray[0];
//        }

//        #region IEnumerable interface

//        /// <summary>
//        /// Gets the enumerator for this PriorityQueue.
//        /// </summary>
//        /// <returns>Enumerator for the current priority queue.</returns>
//        IEnumerator<T> IEnumerable<T>.GetEnumerator()
//        {
//            return new Enumerator(this);
//        }

//        /// <summary>
//        /// Gets the enumerator for this PriorityQueue.
//        /// </summary>
//        /// <returns>Enumerator for the current priority queue.</returns>
//        IEnumerator IEnumerable.GetEnumerator()
//        {
//            return new Enumerator(this);
//        }

//        internal T GetElement(int currentIndex)
//        {
//            return heapArray[currentIndex];
//        }

//        #endregion

//        /// <summary>
//        /// Object which moves through the PriorityQueue collection.
//        /// </summary>
//        public struct Enumerator : IEnumerator<T>
//        {
//            private readonly PriorityQueue<T> queue;
//            private T currentElement;
//            private int currentIndex;

//            internal Enumerator(PriorityQueue<T> q)
//            {
//                queue = q;
//                currentIndex = -1;
//                currentElement = default(T);
//            }

//            #region IEnumerator<T> Members

//            /// <summary>
//            /// Disposes the enumerator.
//            /// </summary>
//            public void Dispose()
//            {
//                currentIndex = -2;
//                currentElement = default(T);
//            }

//            /// <summary>
//            /// Moves the enumerator to the next position of the collection.
//            /// </summary>
//            /// <returns>Whether or not the move succeeded.</returns>
//            public bool MoveNext()
//            {
//                if (currentIndex == -2)
//                {
//                    //This enumerator was previously invalidated.
//                    return false;
//                }
//                currentIndex++;
//                if (currentIndex == queue.count)
//                {
//                    currentIndex = -2;
//                    currentElement = default(T);
//                    return false;
//                }
//                currentElement = queue.GetElement(currentIndex);
//                return true;
//            }

//            /// <summary>
//            /// Gets the enumerator's current position's value.
//            /// </summary>
//            public T Current
//            {
//                get
//                {
//                    if (currentIndex == -1)
//                    {
//                        throw new InvalidOperationException("Can't get the current value, enumeration has not started yet.");
//                    }
//                    if (currentIndex == -2)
//                    {
//                        throw new InvalidOperationException("Can't get the current value, enumeration already ended.");
//                    }

//                    return currentElement;
//                }
//            }

//            /// <summary>
//            /// Gets the enumerator's current position's value.
//            /// </summary>
//            object IEnumerator.Current
//            {
//                get
//                {
//                    if (currentIndex == -1)
//                    {
//                        throw new InvalidOperationException("Can't get the current value, enumeration has not started yet.");
//                    }
//                    if (currentIndex == -2)
//                    {
//                        throw new InvalidOperationException("Can't get the current value, enumeration already ended.");
//                    }

//                    return currentElement;
//                }
//            }

//            /// <summary>
//            /// Moves the enumerator back to the beginning.
//            /// </summary>
//            void IEnumerator.Reset()
//            {
//                currentIndex = -1;
//                currentElement = default(T);
//            }

//            #endregion
//        }
//    }
//}