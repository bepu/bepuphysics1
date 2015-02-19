using System;
using System.Collections.Generic;
using System.Threading;

namespace BEPUutilities.Threading
{
    /// <summary>
    /// Manages parallel for loops.
    /// Cannot handle general task-based parallelism.
    /// </summary>
    public class ParallelLooper : IParallelLooper, IDisposable
    {
        private readonly AutoResetEvent loopFinished;
        private int workerCount;

        internal List<ParallelLoopWorker> workers = new List<ParallelLoopWorker>();

        internal int currentBeginIndex, currentEndIndex;
        internal Action<int> currentLoopBody;
        internal int iterationsPerSteal;

        /// <summary>
        /// Gets or sets the minimum number of tasks to be allocated to each thread
        /// per loop.
        /// </summary>
        public int MinimumTasksPerThread
        {
            get { return minimumTasksPerThread; }
            set { minimumTasksPerThread = value; }
        }

        /// <summary>
        /// Gets or sets the maximum number of loop iterations
        /// per individual task.
        /// </summary>
        public int MaximumIterationsPerTask
        {
            get { return maximumIterationsPerTask; }
            set { maximumIterationsPerTask = value; }
        }

#if WINDOWS
        private int minimumTasksPerThread = 3;
        private int maximumIterationsPerTask = 80;
#else
        int minimumTasksPerThread = 3;
        int maximumIterationsPerTask = 80;
#endif
        internal int jobIndex;
        internal int maxJobIndex;

        /// <summary>
        /// Constructs a new parallel loop manager.
        /// </summary>
        public ParallelLooper()
        {
            loopFinished = new AutoResetEvent(false);
        }

        /// <summary>
        /// Gets the number of threads used by the looper.
        /// </summary>
        public int ThreadCount
        {
            get { return workers.Count; }
        }

        /// <summary>
        /// Adds a thread to the manager.
        /// </summary>
        public void AddThread()
        {
            AddThread(null);
        }

        /// <summary>
        /// Adds a thread to the manager.
        /// </summary>
        /// <param name="threadStart">Initialization to run on the worker thread.</param>
        public void AddThread(Action threadStart)
        {
            workers.Add(new ParallelLoopWorker(this, threadStart));
        }

        /// <summary>
        /// Removes a thread from the manager.
        /// </summary>
        public void RemoveThread()
        {
            if (workers.Count > 0)
            {
                lock (workers[0].disposedLocker)
                {
                    if (!workers[0].disposed)
                    {
                        currentLoopBody = null;
                        workerCount = 1;
                        workers[0].getToWork.Set();
                        loopFinished.WaitOne();
                        workers[0].Dispose();
                    }
                }
                workers.RemoveAt(0);
            }
        }

        /// <summary>
        /// Iterates over the interval.
        /// </summary>
        /// <param name="beginIndex">Starting index of the iteration.</param>
        /// <param name="endIndex">Ending index of the iteration.</param>
        /// <param name="loopBody">Function to call on each iteration.</param>
        public void ForLoop(int beginIndex, int endIndex, Action<int> loopBody)
        {
            //CANNOT CALL THIS WHILE BUSY!!!! ASSUME THAT IS GUARANTEED.
            //Compute intervals for each worker.

            workerCount = workers.Count;

            //TODO: The job splitting could be tuned possibly.
            int iterationCount = endIndex - beginIndex;
            int tasksPerThread = Math.Max(minimumTasksPerThread, iterationCount / maximumIterationsPerTask);
            int taskSubdivisions = workerCount * tasksPerThread;

            currentBeginIndex = beginIndex;
            currentEndIndex = endIndex;
            currentLoopBody = loopBody;
            iterationsPerSteal = Math.Max(1, iterationCount / taskSubdivisions);
            jobIndex = 0;
            float maxJobs = iterationCount / (float) iterationsPerSteal;
            if (maxJobs % 1 == 0)
                maxJobIndex = (int) maxJobs;
            else
                maxJobIndex = 1 + (int) maxJobs;

            for (int i = 0; i < workers.Count; i++)
            {
                workers[i].finalIndex = endIndex;
                workers[i].iterationsPerSteal = iterationsPerSteal;
                workers[i].getToWork.Set();
            }

            loopFinished.WaitOne();
        }

        internal void OnWorkerFinish()
        {
            if (Interlocked.Decrement(ref workerCount) == 0)
                loopFinished.Set();
        }


        private bool disposed;
        private readonly object disposedLocker = new object();

        /// <summary>
        /// Releases resources used by the object.
        /// </summary>
        public void Dispose()
        {
            lock (disposedLocker)
            {
                if (!disposed)
                {
                    disposed = true;
                    while (workers.Count > 0)
                    {
                        RemoveThread();
                    }
                    loopFinished.Close();
                    GC.SuppressFinalize(this);
                }
            }
        }

        /// <summary>
        /// Releases resources used by the object.
        /// </summary>
        ~ParallelLooper()
        {
            Dispose();
        }



    }
}