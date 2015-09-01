using System;
using System.Diagnostics;
using BEPUutilities.ResourceManagement;
using System.Collections.Generic;
#if FORCEINLINE
using System.Runtime.CompilerServices;
#endif

namespace BEPUutilities.DataStructures
{
    /// <summary>
    /// Container supporting double ended queue behaviors built on top of pooled arrays.
    /// </summary>
    /// <remarks>Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care, it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, it rarely checks input for errors, and the enumerator doesn't check for mid-enumeration modification.</remarks>
    /// <typeparam name="T">Type of the elements in the queue.</typeparam>
    public struct QuickQueue<T> : IDisposable, IEnumerable<T>
    {
        private int poolIndex;
        private BufferPool<T> pool;
        int capacityMask;


        /// <summary>
        /// Gets the backing array containing the elements of the queue.
        /// Indices from FirstIndex to LastIndex inclusive hold actual data. All other data is undefined.
        /// Watch out for wrap around; LastIndex can be less than FirstIndex even when count > 0!
        /// </summary>
        public readonly T[] Elements;

        int firstIndex;
        /// <summary>
        /// Gets the index of the first element in the queue.
        /// </summary>
        public int FirstIndex
        {
            get
            {
                return firstIndex;
            }
        }

        int lastIndex;
        /// <summary>
        /// Gets the index of the last element in the queue.
        /// </summary>
        public int LastIndex
        {
            get
            {
                return lastIndex;
            }
        }

        private int count;
        /// <summary>
        /// Gets the number of elements in the queue.
        /// </summary>
        public int Count
        {
            get { return count; }
        }



        /// <summary>
        /// Gets an element at the given index in the queue.
        /// 0 gets the element at the FirstIndex. Count-1 would get the element at LastIndex.
        /// </summary>
        /// <param name="index">Index to grab an element from.</param>
        /// <returns>Element at the given index in the queue.</returns>
        public T this[int index]
        {
            //You would think that such a trivial accessor would inline without any external suggestion.
            //Sometimes, yes. Sometimes, no. :(
#if FORCEINLINE
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
            get
            {
                ValidateIndex(index);
                return Elements[(FirstIndex + index) & capacityMask];
            }
#if FORCEINLINE
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
            set
            {
                ValidateIndex(index);
                Elements[(FirstIndex + index) & capacityMask] = value;
            }
        }

        /// <summary>
        /// Creates a new queue.
        /// </summary>
        /// <param name="pool">Pool from which to retrieve typed arrays.</param>
        /// <param name="initialPoolIndex">Initial pool index to pull the backing array from. The size of the initial buffer will be 2^initialPoolIndex.</param>
        public QuickQueue(BufferPool<T> pool, int initialPoolIndex = 5)
        {
            this.pool = pool;
            poolIndex = initialPoolIndex;
            Elements = pool.TakeFromPoolIndex(poolIndex);
            count = 0;
            capacityMask = Elements.Length - 1;
            firstIndex = 0;
            lastIndex = capacityMask; //length - 1

        }

        private void Resize(int newPoolIndex)
        {
            Debug.Assert(count <= (1 << newPoolIndex), "New pool index must contain all elements in the queue.");
            var oldQueue = this;
            this = new QuickQueue<T>(pool, newPoolIndex);
            count = oldQueue.Count;
            //Copy the old first-end to the first part of the new array.
            Array.Copy(oldQueue.Elements, oldQueue.firstIndex, Elements, 0, oldQueue.Elements.Length - oldQueue.firstIndex);
            //Copy the old begin-first to the second part of the new array.
            Array.Copy(oldQueue.Elements, 0, Elements, oldQueue.Elements.Length - oldQueue.firstIndex, oldQueue.firstIndex);

            firstIndex = 0;
            lastIndex = count - 1;

            oldQueue.Dispose();
        }

        /// <summary>
        /// Enqueues the element to the end of the queue, incrementing the last index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void Enqueue(T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(lastIndex = ((lastIndex + 1) & capacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Enqueues the element to the start of the queue, decrementing the first index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void EnqueueFirst(T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(firstIndex = ((firstIndex - 1) & capacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Enqueues the element to the end of the queue, incrementing the last index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void Enqueue(ref T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(lastIndex = ((lastIndex + 1) & capacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Enqueues the element to the start of the queue, decrementing the first index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void EnqueueFirst(ref T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(firstIndex = ((firstIndex - 1) & capacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Dequeues an element from the start of the queue, incrementing the first index.
        /// </summary>
        /// <returns>Element removed from the queue.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public T Dequeue()
        {
            Validate();
            if (count == 0)
                throw new InvalidOperationException("The queue is empty.");
            var element = Elements[firstIndex];
            Elements[firstIndex] = default(T);
            firstIndex = (firstIndex + 1) & capacityMask;
            --count;
            return element;

        }

        /// <summary>
        /// Dequeues an element from the end of the queue, decrementing the last index.
        /// </summary>
        /// <returns>Element removed from the queue.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public T DequeueLast()
        {
            Validate();
            if (count == 0)
                throw new InvalidOperationException("The queue is empty.");
            var element = Elements[lastIndex];
            Elements[lastIndex] = default(T);
            lastIndex = (lastIndex - 1) & capacityMask;
            --count;
            return element;

        }

        /// <summary>
        /// Attempts to dequeue an element from the start of the queue, incrementing the first index.
        /// </summary>
        /// <param name="element">Element removed from the queue, if any.</param>
        /// <returns>True if an element was available to remove, false otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public bool TryDequeue(out T element)
        {
            Validate();
            if (count > 0)
            {
                element = Elements[firstIndex];
                Elements[firstIndex] = default(T);
                firstIndex = (firstIndex + 1) & capacityMask;
                --count;
                return true;
            }
            element = default(T);
            return false;

        }

        /// <summary>
        /// Attempts to dequeue an element from the end of the queue, decrementing the last index.
        /// </summary>
        /// <param name="element">Element removed from the queue, if any.</param>
        /// <returns>True if an element was available to remove, false otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public bool TryDequeueLast(out T element)
        {
            Validate();
            if (count > 0)
            {
                element = Elements[lastIndex];
                Elements[lastIndex] = default(T);
                lastIndex = (lastIndex - 1) & capacityMask;
                --count;
                return true;
            }
            element = default(T);
            return false;

        }

        /// <summary>
        /// Copies the elements of the <see cref="T:System.Collections.Generic.ICollection`1"/> to an <see cref="T:System.Array"/>, starting at a particular <see cref="T:System.Array"/> index.
        /// </summary>
        /// <param name="array">The one-dimensional <see cref="T:System.Array"/> that is the destination of the elements copied from <see cref="T:System.Collections.Generic.ICollection`1"/>. The <see cref="T:System.Array"/> must have zero-based indexing.</param><param name="arrayIndex">The zero-based index in <paramref name="array"/> at which copying begins.</param><exception cref="T:System.ArgumentNullException"><paramref name="array"/> is null.</exception><exception cref="T:System.ArgumentOutOfRangeException"><paramref name="arrayIndex"/> is less than 0.</exception><exception cref="T:System.ArgumentException">The number of elements in the source <see cref="T:System.Collections.Generic.ICollection`1"/> is greater than the available space from <paramref name="arrayIndex"/> to the end of the destination <paramref name="array"/>.</exception>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void CopyTo(T[] array, int arrayIndex)
        {
            if (count > 0)
            {
                if (firstIndex <= lastIndex)
                {
                    Array.Copy(Elements, firstIndex, array, arrayIndex, count);
                }
                else
                {
                    //Copy the old first-end to the first part of the new array.
                    Array.Copy(Elements, firstIndex, array, arrayIndex, Elements.Length - firstIndex);
                    //Copy the old begin-last to the second part of the new array.
                    Array.Copy(Elements, 0, array, arrayIndex + Elements.Length - firstIndex, lastIndex + 1);
                }
            }
        }

        /// <summary>
        /// Clears the queue by setting the count to zero and explicitly setting all relevant indices in the backing array to default values.
        /// </summary>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void Clear()
        {
            Validate();
            if (lastIndex >= firstIndex)
            {
                Array.Clear(Elements, firstIndex, count);
            }
            else if (count > 0)
            {
                Array.Clear(Elements, firstIndex, Elements.Length - firstIndex);
                Array.Clear(Elements, 0, lastIndex + 1);
            }
            count = 0;
            firstIndex = 0;
            lastIndex = capacityMask; //length - 1
        }

        /// <summary>
        /// Clears the queue without changing any of the values in the backing array. Be careful about using this if the queue contains reference types.
        /// </summary>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void FastClear()
        {
            count = 0;
            firstIndex = 0;
            lastIndex = capacityMask;
        }

        /// <summary>
        /// Compacts the internal buffer to the minimum size required for the number of elements in the queue.
        /// </summary>
        public void Compact()
        {
            Validate();
            var newPoolIndex = BufferPool<T>.GetPoolIndex(Count);
            if (newPoolIndex != poolIndex)
                Resize(newPoolIndex);
        }

        /// <summary>
        /// Clears and returns the queue's buffers.
        /// </summary>
        public void Dispose()
        {
            Clear();
            pool.GiveBack(Elements, poolIndex);
#if DEBUG
            pool = null;
#endif
        }

        [Conditional("DEBUG")]
        void ValidateIndex(int index)
        {
            Debug.Assert(index >= 0 && index < Count, "Index must be nonnegative and less than the number of elements in the queue.");
        }


        [Conditional("DEBUG")]
        private void Validate()
        {
            Debug.Assert(pool != null, "Should not use a default-constructed or disposed QuickQueue.");
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(Elements, count, firstIndex);
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return GetEnumerator();
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public struct Enumerator : IEnumerator<T>
        {
            private readonly T[] backingArray;
            private readonly int count;
            private readonly int firstIndex;
            private readonly int capacityMask;
            private int index;

            public Enumerator(T[] backingArray, int count, int firstIndex)
            {
                this.backingArray = backingArray;
                this.count = count;
                this.firstIndex = firstIndex;
                this.capacityMask = backingArray.Length - 1;
                index = -1;
            }

            public T Current
            {
                get { return backingArray[(firstIndex + index) & capacityMask]; }
            }

            public void Dispose()
            {
            }

            object System.Collections.IEnumerator.Current
            {
                get { return Current; }
            }

            public bool MoveNext()
            {
                return ++index < count;
            }

            public void Reset()
            {
                index = -1;
            }
        }
    }
}
