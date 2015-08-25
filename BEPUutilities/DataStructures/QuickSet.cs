using System;
using System.Collections.Generic;
using System.Diagnostics;
using BEPUutilities.ResourceManagement;
#if FORCEINLINE
using System.Runtime.CompilerServices;
#endif

namespace BEPUutilities.DataStructures
{
    /// <summary>
    /// Container supporting constant time adds and removes while preserving fast iteration times.
    /// Offers very direct access to information at the cost of safety.
    /// </summary>
    /// <remarks><para>Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care, it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, it is particularly vulnerable to bad hash functions, 
    /// it rarely checks input for errors, and the enumerator doesn't check for mid-enumeration modification.</para>
    /// <para>Note that the implementation is extremely simple. It uses single-step linear probing under the assumption of very low collision rates.
    /// A generous table capacity is recommended; this trades some memory for simplicity and runtime performance.</para></remarks>
    /// <typeparam name="T">Type of element held by the container.</typeparam>
    public struct QuickSet<T> : IDisposable, IEnumerable<T> where T : IEquatable<T>
    {
        private int count;
        /// <summary>
        /// Gets the number of elements in the set.
        /// </summary>
        public int Count
        {
            get { return count; }
        }

        private int[] table;

        /// <summary>
        /// Gets the backing array containing the elements of the set.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public readonly T[] Elements;



        private readonly BufferPool<int> tablePool;
        private readonly BufferPool<T> elementPool;

        /// <summary>
        /// Gets or sets an element at the given index in the list representation.
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
                Debug.Assert(index >= 0 && index < count, "Index should be within the list's size.");
                return Elements[index];
            }
#if FORCEINLINE
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
            set
            {
                Debug.Assert(index >= 0 && index < count, "Index should be within the list's size.");
                Elements[index] = value;
            }
        }

        private int elementPoolIndex;
        private int tablePoolIndex;
        private int tableMask;



        /// <summary>
        /// Creates a new set.
        /// </summary>
        /// <param name="tablePool">Pool from which to retrieve integer arrays.</param>
        /// <param name="elementPool">Pool from which to retrieve typed arrays.</param>
        /// <param name="initialElementPoolIndex">Initial pool index to pull the object buffer from. The size of the initial buffer will be 2^initialElementPoolIndex.</param>
        /// <param name="tableSizePower">Initial pool index to pull the object buffer from. The size of the initial table buffer will be 2^(initialElementPoolIndex + tableSizePower).</param>
        public QuickSet(BufferPool<T> elementPool, BufferPool<int> tablePool, int initialElementPoolIndex = 2, int tableSizePower = 3)
        {
            if (tableSizePower <= 0)
                throw new ArgumentException("The hash table must be larger than the element array.", "tableSizePower");
            if (initialElementPoolIndex < 0)
                throw new ArgumentException("Initial pool index must be nonnegative.", "initialElementPoolIndex");
            this.tablePool = tablePool;
            this.elementPool = elementPool;

            elementPoolIndex = initialElementPoolIndex;
            tablePoolIndex = initialElementPoolIndex + tableSizePower;
            Elements = elementPool.TakeFromPoolIndex(elementPoolIndex);
            table = tablePool.TakeFromPoolIndex(tablePoolIndex);
            //Correctness requires a clean table. '0' means 'not taken'.
            Array.Clear(table, 0, table.Length);
            count = 0;
            tableMask = table.Length - 1;

        }

        private void Resize(int newObjectPoolIndex, int newTablePoolIndex)
        {
            //Just double the size of the set.
            var oldSet = this;
            this = new QuickSet<T>(elementPool, tablePool, newObjectPoolIndex, newTablePoolIndex - newObjectPoolIndex);
            for (int i = oldSet.count - 1; i >= 0; --i)
            {
                Add(oldSet.Elements[i]);
            }
            oldSet.Dispose();
        }

        /// <summary>
        /// Gets the index of the element in the table.
        /// </summary>
        /// <param name="element">Element to look up.</param>
        /// <param name="tableIndex">Index of the element in the redirect table, or if it is not present, the index of where it would be added.</param>
        /// <param name="elementIndex">The index of the element in the elements array, if it exists; -1 otherwise.</param>
        /// <returns>True if the element is present in the set, false if it is not.</returns>
#if FORCEINLINE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
#endif
        private bool GetIndices(T element, out int tableIndex, out int elementIndex)
        {
            //The table lengths are guaranteed to be a power of 2, so the modulo is a simple binary operation.
            //TODO: forcing an inline on this function would be useful on platforms where it is possible.
            tableIndex = element.GetHashCode() & tableMask;
            //0 in the table means 'not taken'; all other values are offset by 1 upward. That is, 1 is actually index 0, 2 is actually index 1, and so on.
            //This is preferred over using a negative number for flagging since clean buffers will contain all 0's.
            while ((elementIndex = table[tableIndex]) > 0)
            {
                //This table index is taken. Is this the specified element?
                //Remember to decode the object index.
                if (Elements[--elementIndex].Equals(element))
                {
                    return true;
                }
                tableIndex = (tableIndex + 1) & tableMask;
            }
            elementIndex = -1;
            return false;
        }

        /// <summary>
        /// Gets the index of the element in the list if it exists.
        /// </summary>
        /// <param name="element">Element to get the index of.</param>
        /// <returns>The index of the element if the element exists in the list, -1 otherwise.</returns>
        public int IndexOf(T element)
        {
            Validate();
            int tableIndex, objectIndex;
            GetIndices(element, out tableIndex, out objectIndex);
            return objectIndex;
        }

        /// <summary>
        /// Checks if a given element already belongs to the set.
        /// </summary>
        /// <param name="element">Element to test for.</param>
        /// <returns>True if the element already belongs to the set, false otherwise.</returns>
        public bool Contains(T element)
        {
            Validate();
            int tableIndex, objectIndex;
            return GetIndices(element, out tableIndex, out objectIndex);
        }

        /// <summary>
        /// Adds an element to the set. If a version of the element (same hash code, 'equal' by comparer) is already present,
        /// it is replaced by the given version.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present and was instead replaced.</returns>
        public bool AddAndReplace(T element)
        {
            Validate();
            if (count == Elements.Length)
            {
                //There's no room left; resize.
                Resize(elementPoolIndex + 1, tablePoolIndex + 1);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }

            int tableIndex, elementIndex;
            if (GetIndices(element, out tableIndex, out elementIndex))
            {
                //Already present!
                Elements[elementIndex] = element;
                return false;
            }


            //It wasn't in the set. Add it!
            Elements[count] = element;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            table[tableIndex] = ++count;
            return true;
        }


        /// <summary>
        /// Adds an element to the set if it is not already present.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present.</returns>
        public bool Add(T element)
        {
            Validate();

            if (count == Elements.Length)
            {
                //There's no room left; resize.
                Resize(elementPoolIndex + 1, tablePoolIndex + 1);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }

            int tableIndex, elementIndex;
            if (GetIndices(element, out tableIndex, out elementIndex))
            {
                //Already present!
                return false;
            }


            //It wasn't in the set. Add it!
            Elements[count] = element;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            table[tableIndex] = ++count;
            return true;
        }

        //Note: the reason this is named "FastRemove" instead of just "Remove" despite it being the only remove present is that
        //there may later exist an order preserving "Remove". That would be a very sneaky breaking change.

        /// <summary>
        /// Removes an element from the set if belongs to the set.
        /// Does not preserve the order of elements in the set.
        /// </summary>
        /// <param name="element">Element to remove.</param>
        /// <returns>True if the element was found and removed, false otherwise.</returns>
        public bool FastRemove(T element)
        {
            Validate();
            //Find it.
            int tableIndex, objectIndex;
            if (GetIndices(element, out tableIndex, out objectIndex))
            {
                //We found the object!
                //Add and remove must both maintain a property:
                //All items are either at their desired index (as defined by the hash), or they are contained in a contiguous block clockwise from the desired index.
                //Removals seek to fill the gap they create by searching clockwise to find items which can be moved backward.
                //Search clockwise for an item to fill this slot. The search must continue until a gap is found.
                int moveCandidateIndex;
                int gapIndex = tableIndex;
                //Search clockwise.
                while ((moveCandidateIndex = table[tableIndex = (tableIndex + 1) & tableMask]) > 0)
                {
                    //This slot contains something. What is its actual index?
                    --moveCandidateIndex;
                    int desiredIndex = Elements[moveCandidateIndex].GetHashCode() & tableMask;

                    //Would this element be closer to its actual index if it was moved to the gap?
                    //To find out, compute the clockwise distance from the gap and the clockwise distance from the ideal location.

                    var distanceFromGap = (tableIndex - gapIndex) & tableMask;
                    var distanceFromIdeal = (tableIndex - desiredIndex) & tableMask;
                    if (distanceFromGap <= distanceFromIdeal)
                    {
                        //The distance to the gap is less than or equal the distance to the ideal location, so just move to the gap.
                        table[gapIndex] = table[tableIndex];
                        gapIndex = tableIndex;
                    }

                }
                //Clear the table gap left by the removal.
                table[gapIndex] = 0;
                //Swap the final element into the removed object's element array index, if the removed object wasn't the last object.
                --count;
                if (objectIndex < count)
                {
                    Elements[objectIndex] = Elements[count];
                    //Locate the swapped object in the table and update its index.
                    int oldObjectIndex;
                    GetIndices(Elements[objectIndex], out tableIndex, out oldObjectIndex);
                    table[tableIndex] = objectIndex + 1; //Remember the encoding! all indices offset by 1.
                }
                //Clear the final slot in the elements set.
                Elements[count] = default(T);

                return true;
            }
            return false;
        }


        /// <summary>
        /// Shrinks the internal buffers to the smallest acceptable size and releases the old buffers to the pools.
        /// </summary>
        public void Compact()
        {
            Validate();
            var minimumRequiredPoolIndex = BufferPool<T>.GetPoolIndex(count);
            if (minimumRequiredPoolIndex != elementPoolIndex)
                Resize(minimumRequiredPoolIndex, minimumRequiredPoolIndex + (tablePoolIndex - elementPoolIndex));
        }

        /// <summary>
        /// Removes all elements from the set.
        /// </summary>
        public void Clear()
        {
            //While it may be appealing to remove individual elements from the set when the set is sparse,
            //using a brute force clear over the entire table is almost always faster. And it's a lot simpler!
            Array.Clear(table, 0, table.Length);
            Array.Clear(Elements, 0, count);
            count = 0;
        }

        /// <summary>
        /// Removes all elements from the set without modifying the contents of the elements array. Be careful about using this with reference types.
        /// </summary>
        public void FastClear()
        {
            Array.Clear(table, 0, table.Length);
            count = 0;
        }

        public Enumerator GetEnumerator()
        {
            Validate();
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

        [Conditional("DEBUG")]
        private void Validate()
        {
            Debug.Assert(table != null, "The QuickSet must have its internal buffers and pools available; default-constructed or disposed QuickSets should not be used.");
        }

        /// <summary>
        /// Clears and returns the set's buffers to the pools.
        /// </summary>
        public void Dispose()
        {
            //We must clean out the table before returning it to the pool in case the array contains reference types which would otherwise leak.
            //The user may have already manually cleared it. To avoid doing redundant work, check the count first.
            //The user may have chosen to leave reference types in the list if they did a fast clear, but that's not for us to worry about.
            if (count > 0)
            {
                Clear();
            }

            tablePool.GiveBack(table, tablePoolIndex);
            elementPool.GiveBack(Elements, elementPoolIndex);
#if DEBUG
            table = null;
#endif
        }



    }
}
