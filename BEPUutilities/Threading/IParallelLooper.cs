using System;

namespace BEPUutilities.Threading
{

    /// <summary>
    /// Provides a multithreaded for loop interface.
    /// </summary>
    public interface IParallelLooper
    {

        /// <summary>
        /// Loops from the starting index (inclusive) to the ending index (exclusive), calling the loopBody at each iteration.
        /// The forLoop function will not return until all iterations are complete.
        /// This is meant to be used in a 'fork-join' model; only a single thread should be running a forLoop
        /// at any time.
        /// </summary>
        /// <param name="startIndex">Inclusive starting index.</param>
        /// <param name="endIndex">Exclusive ending index.</param>
        /// <param name="loopBody">Function that handles an individual iteration of the loop.</param>
        void ForLoop(int startIndex, int endIndex, Action<int> loopBody);

        /// <summary>
        /// Gets the number of threads available to the loop.
        /// This is used to configure job sizes where applicable.
        /// It does not have to be exactly correct; an estimate will work.
        /// Better estimates should result in better performance.
        /// </summary>
        int ThreadCount { get; }
    }
}