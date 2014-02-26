using System;
using System.Diagnostics;
using BEPUutilities.ResourceManagement;
using System.Collections.Generic;

namespace BEPUutilities.DataStructures
{
    /// <summary>
    /// Container supporting list-like behaviors built on top of pooled arrays.
    /// </summary>
    /// <remarks>Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care, it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, it rarely checks input for errors, and the enumerator doesn't check for mid-enumeration modification.</remarks>
    /// <typeparam name="T">Type of the elements in the list.</typeparam>
    public struct QuickList<T> : IDisposable, IEnumerable<T>
    {
        private int poolIndex;
        private BufferPool<T> pool;

        /// <summary>
        /// Gets the backing array containing the elements of the list.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public readonly T[] Elements;

#if DEBUG
        private int count;
        /// <summary>
        /// Gets or sets the number of pairs in the list.
        /// </summary>
        public int Count
        {
            get { return count; }
            set
            {
                Debug.Assert(value >= 0 && value < Elements.Length, "Count should fit in the current backing array length.");
                count = value;
            }
        }
#else
        /// <summary>
        /// Gets or sets the number of pairs in the list.
        /// </summary>
        public int Count;
#endif

        /// <summary>
        /// Gets an element at the given index in the list.
        /// </summary>
        /// <param name="index">Index to grab an element from.</param>
        /// <returns>Element at the given index in the list.</returns>
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
                return Elements[index];
            }
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
            set
            {
                ValidateIndex(index);
                Elements[index] = value;
            }
        }


        /// <summary>
        /// Creates a new list.
        /// </summary>
        /// <param name="pool">Pool from which to retrieve typed arrays.</param>
        /// <param name="initialPoolIndex">Initial pool index to pull the backing array from. The size of the initial buffer will be 2^initialPoolIndex.</param>
        public QuickList(BufferPool<T> pool, int initialPoolIndex = 2)
        {
            this.pool = pool;
            poolIndex = initialPoolIndex;
            Elements = pool.TakeFromPoolIndex(poolIndex);
#if DEBUG
            count = 0;
#else
            Count = 0;
#endif

        }

        private void Resize(int newPoolIndex)
        {
            Debug.Assert(Count <= (1 << newPoolIndex), "New pool index must contain all elements in the list.");
            var oldList = this;
            this = new QuickList<T>(pool, newPoolIndex);
            Count = oldList.Count;
            Array.Copy(oldList.Elements, Elements, Count);
            oldList.Dispose();
        }

        /// <summary>
        /// Adds the element to the list.
        /// </summary>
        /// <param name="element">Item to add.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void Add(T element)
        {
            Validate();
            if (Count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Adds the element to the list without checking the count against the capacity.
        /// </summary>
        /// <param name="element">Item to add.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void AddUnsafely(T element)
        {
            Validate();
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Adds the element to the list.
        /// </summary>
        /// <param name="element">Element to add.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void Add(ref T element)
        {
            Validate();
            if (Count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Adds the element to the list without checking the count against the capacity.
        /// </summary>
        /// <param name="element">Element to add.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void AddUnsafely(ref T element)
        {
            Validate();
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Gets the index of the element in the list, if present.
        /// </summary>
        /// <param name="element">Element to find.</param>
        /// <returns>Index of the element in the list if present, -1 otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public int IndexOf(T element)
        {
            Validate();
            return Array.IndexOf(Elements, element, 0, Count);
        }

        /// <summary>
        /// Gets the index of the element in the list, if present.
        /// </summary>
        /// <param name="element">Element to find.</param>
        /// <returns>Index of the element in the list if present, -1 otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public int IndexOf(ref T element)
        {
            Validate();
            return Array.IndexOf(Elements, element, 0, Count);
        }

        /// <summary>
        /// Removes an element from the list. Preserves the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public bool Remove(T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                RemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list. Preserves the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public bool Remove(ref T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                RemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list. Does not preserve the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public bool FastRemove(T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                FastRemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list. Does not preserve the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public bool FastRemove(ref T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                FastRemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list at the given index. Preserves the order of elements.
        /// </summary>
        /// <param name="index">Index of the element to remove from the list.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void RemoveAt(int index)
        {
            Validate();
            ValidateIndex(index);
            --Count;
            if (index < Count)
            {
                //Copy everything from the removal point onward backward one slot.
                Array.Copy(Elements, index + 1, Elements, index, Count - index);
            }
            //Clear out the former last slot.
            Elements[Count] = default(T);
        }

        /// <summary>
        /// Removes an element from the list at the given index. Does not preserve the order of elements.
        /// </summary>
        /// <param name="index">Index of the element to remove from the list.</param>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void FastRemoveAt(int index)
        {
            Validate();
            ValidateIndex(index);
            --Count;
            if (index < Count)
            {
                //Put the final element in the removed slot.
                Elements[index] = Elements[Count];
            }
            //Clear out the former last slot.
            Elements[Count] = default(T);
        }


        /// <summary>
        /// Removes and outputs the last element in the list if it exists.
        /// </summary>
        /// <param name="element">Last element of the list.</param>
        /// <returns>True if the element existed and was removed, false otherwise.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public bool TryPop(out T element)
        {
            Validate();
            if (Count > 0)
            {
                Count--;
                element = Elements[Count];
                return true;
            }
            element = default(T);
            return false;
        }

        /// <summary>
        /// Clears the list by setting the count to zero and explicitly setting all relevant indices in the backing array to default values.
        /// </summary>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        public void Clear()
        {
            Validate();
            Array.Clear(Elements, 0, Count);
            Count = 0;
        }

        /// <summary>
        /// Compacts the internal buffer to the minimum size required for the number of elements in the list.
        /// </summary>
        public void Compact()
        {
            Validate();
            var newPoolIndex = BufferPool<T>.GetPoolIndex(Count);
            if (newPoolIndex != poolIndex)
                Resize(newPoolIndex);
        }

        /// <summary>
        /// Clears and returns the list's buffers.
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
            Debug.Assert(index >= 0 && index < Count, "Index must be nonnegative and less than the number of elements in the list.");
        }


        [Conditional("DEBUG")]
        private void Validate()
        {
            Debug.Assert(pool != null, "Should not use a default-constructed or disposed QuickList.");
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(Elements, Count);
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
            private int index;

            public Enumerator(T[] backingArray, int count)
            {
                this.backingArray = backingArray;
                this.count = count;
                index = -1;
            }

            public T Current
            {
                get { return backingArray[index]; }
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
