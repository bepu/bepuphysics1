using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace BEPUutilities.ResourceManagement
{
    /// <summary>
    /// Provides storage for reusable arrays with power-of-2 lengths.
    /// </summary>
    /// <typeparam name="T">Type of resource contained in the buffers.</typeparam>
    /// <remarks>This is designed for use with unsafe code. It often sacrifices safety for performance or simplicity.
    /// Running with DEBUG defined will catch some misuse, but otherwise many invalid usages will be allowed.</remarks>
    public class BufferPool<T>
    {
        /// <summary>
        /// Defines the maximum buffer size. Maximum length is 2^MaximumPoolIndex.
        /// </summary>
        private const int MaximumPoolIndex = 30;
        private Stack<T[]>[] pools = new Stack<T[]>[MaximumPoolIndex + 1];
#if DEBUG
        private HashSet<T[]> outstandingResources = new HashSet<T[]>();
#endif

        /// <summary>
        /// Constructs a new resource buffer pool.
        /// </summary>
        public BufferPool()
        {
            for (int i = 0; i < pools.Length; ++i)
            {
                pools[i] = new Stack<T[]>();
            }
        }

        /// <summary>
        /// Gets the exponent associated with the buffer pool which would hold the given count of elements.
        /// </summary>
        /// <param name="count">Element count to compute the batch index of.</param>
        /// <returns>Exponent associated with the buffer pool which would hold the given count of elements.</returns>
        public static int GetPoolIndex(int count)
        {
            Debug.Assert(count >= 0 && count < (1 << MaximumPoolIndex), "Count must be from 0 to " + ((1 << MaximumPoolIndex) - 1) + ", inclusive.");
            //We want the buffer which would fully contain the count, so it should be effectively Ceiling(Log(count)).
            //Doubling the value (and subtracting one, to avoid the already-a-power-of-two case) takes care of this.
            count = ((count > 0 ? count : 1) << 1) - 1;
            int log = 0;
            if ((count & 0xFFFF0000) > 0)
            {
                count >>= 16;
                log |= 16;
            }
            if ((count & 0xFF00) > 0)
            {
                count >>= 8;
                log |= 8;
            }
            if ((count & 0xF0) > 0)
            {
                count >>= 4;
                log |= 4;
            }
            if ((count & 0xC) > 0)
            {
                count >>= 2;
                log |= 2;
            }
            if ((count & 0x2) > 0)
            {
                log |= 1;
            }
            return log;
        }

        /// <summary>
        /// Takes a buffer from the given pool index.
        /// </summary>
        /// <param name="poolIndex">Pool to grab a buffer from.</param>
        /// <returns>Pool of the requested size.</returns>
        public virtual T[] TakeFromPoolIndex(int poolIndex)
        {
            Debug.Assert(poolIndex >= 0 && poolIndex <= MaximumPoolIndex, "Pool index should be from 0 to " + MaximumPoolIndex + " inclusive.");
            T[] toReturn;
            if (pools[poolIndex].Count > 0)
            {
                toReturn = pools[poolIndex].Pop();
            }
            else
                toReturn = new T[1 << poolIndex];
#if DEBUG
            outstandingResources.Add(toReturn);
#endif
            return toReturn;
        }

        /// <summary>
        /// Grabs a buffer of sufficient size to hold the given number of elements.
        /// </summary>
        /// <param name="minimumSize">Number of elements that must be able to fit in the buffer.</param>
        /// <returns>Buffer of sufficient size to hold the given number of elements.</returns>
        public T[] Take(int minimumSize)
        {
            return TakeFromPoolIndex(GetPoolIndex(minimumSize));
        }


        /// <summary>
        /// Releases a buffer back to the pool without clearing it out.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        /// <param name="poolIndex">Pool index associated with the buffer.</param>
        public virtual void GiveBack(T[] buffer, int poolIndex)
        {
#if DEBUG
            Debug.Assert(outstandingResources.Remove(buffer), "The buffer being returned must come from this pool, and buffers should only be returned once.");
            if (CheckIfReturnedBuffersAreClean)
            {
                for (int i = 0; i < buffer.Length; ++i)
                {
                    Debug.Assert(EqualityComparer<T>.Default.Equals(buffer[i], default(T)), "Buffers being returned to the pool should be clean. Every index should hold default(T).");
                }
            }
#endif
            pools[poolIndex].Push(buffer);
        }

        /// <summary>
        /// Gives a buffer back to the pool without clearing it out.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        public void GiveBack(T[] buffer)
        {
            GiveBack(buffer, GetPoolIndex(buffer.Length));
        }

        /// <summary>
        /// Ensures that there are at least the specified number of buffers allocated for the given batch index.
        /// </summary>
        /// <param name="poolIndex">Pool index to ensure the count of.</param>
        /// <param name="count">Minimum number of elements that need to exist in the specified pool.</param>
        public void EnsureBufferCount(int poolIndex, int count)
        {
            while (pools[poolIndex].Count < count)
                pools[poolIndex].Push(new T[1 << poolIndex]);
        }

        /// <summary>
        /// Drops all references held by the pool.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i <= MaximumPoolIndex; ++i)
            {
                pools[i].Clear();
            }
        }


#if DEBUG
        /// <summary>
        /// Gets or sets whether to check buffers for cleanliness when they are returned when compiled in debug mode. A clean buffer is defined as one that contains all default values.
        /// </summary>
        public bool CheckIfReturnedBuffersAreClean { get; set; }
#endif
    }
}
