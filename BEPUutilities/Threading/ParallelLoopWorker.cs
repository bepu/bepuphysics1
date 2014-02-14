using System;
using System.Threading;

namespace BEPUutilities.Threading
{
    internal class ParallelLoopWorker : IDisposable
    {
        private readonly ParallelLooper manager;
        internal bool disposed;
        internal object disposedLocker = new object();
        internal int finalIndex;

        internal AutoResetEvent getToWork;
        
        internal int iterationsPerSteal;
        private Thread thread;
        private Action threadStart;

        internal ParallelLoopWorker(ParallelLooper manager, Action threadStart)
        {
            this.manager = manager;
            this.threadStart = threadStart;

            getToWork = new AutoResetEvent(false);

            thread = new Thread(Work) { IsBackground = true };
            thread.Start();
        }





        internal void Work()
        {
            if (threadStart != null)
            {
                threadStart();
            }
            threadStart = null;

            while (true)
            {
                //When the owning ParallelLooper is told to start a loop, it will notify the worker via this signal.
                getToWork.WaitOne();
                if (manager.currentLoopBody == null)
                {
                    //Woops, looks like it's time for me to die.
                    manager.OnWorkerFinish();
                    return;
                }

                while (manager.jobIndex <= manager.maxJobIndex)
                {
                    //Claim a piece of job.
                    int jobIndex = Interlocked.Increment(ref manager.jobIndex);
                    //The job interval.
                    int endIndex = jobIndex * iterationsPerSteal;
                    int beginIndex = endIndex - iterationsPerSteal;

                    //Do the job piece.  Make sure you don't do more than exists in the list itself.
                    for (int i = beginIndex; i < endIndex && i < finalIndex; i++)
                    {
                        manager.currentLoopBody(i);
                    }
                } //this is not 'thread safe' but the result of the unsafety is a quick fail in the worst case.

                manager.OnWorkerFinish();
            }
        }



        /// <summary>
        /// Disposes the worker.
        /// </summary>
        public void Dispose()
        {
            lock (disposedLocker)
            {
                if (!disposed)
                {
                    disposed = true;
                    getToWork.Dispose();
                    getToWork = null;
                    thread = null;
                    GC.SuppressFinalize(this);
                }
            }
        }

        /// <summary>
        /// Releases resources used by the object.
        /// </summary>
        ~ParallelLoopWorker()
        {
            Dispose();
        }
    }
}