using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace BEPUutilities.ResourceManagement
{
    /// <summary>
    /// Provides storage for reusable arrays with power-of-2 lengths.
    /// Provides thread safe TakeFromPoolIndex, Take, and GiveBack implementations.
    /// </summary>
    /// <typeparam name="T">Type of resource contained in the buffers.</typeparam>
    public class LockingBufferPool<T> : BufferPool<T>
    {
        private SpinLock locker = new SpinLock();

        /// <summary>
        /// Takes a buffer from the given pool index.
        /// </summary>
        /// <param name="poolIndex">Pool to grab a buffer from.</param>
        /// <returns>Pool of the requested size.</returns>
        public override T[] TakeFromPoolIndex(int poolIndex)
        {
            locker.Enter();
            var toReturn = base.TakeFromPoolIndex(poolIndex);
            locker.Exit();
            return toReturn;
        }
        
        /// <summary>
        /// Releases a buffer back to the pool without clearing it out.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        /// <param name="poolIndex">Pool index associated with the buffer.</param>
        public override void GiveBack(T[] buffer, int poolIndex)
        {
            locker.Enter();
            base.GiveBack(buffer, poolIndex);
            locker.Exit();
        }


    }
}
