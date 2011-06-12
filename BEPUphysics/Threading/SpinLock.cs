using System.Threading;

namespace BEPUphysics.Threading
{
    /// <summary>
    /// Synchronizes using a busy wait.  Take care when using this; if the critical section is long or there's any doubt about the use of a busy wait, consider using Monitor locks or other approaches instead.
    /// Replaces the .NET SpinLock on PC and provides its functionality on the Xbox360.
    /// </summary>
    public class SpinLock
    {
        private const int SleepInterval = 10;
        private int owner = -1;


        /// <summary>
        /// Enters the critical section.  This is not reentrant.
        /// </summary>
        public void Enter()
        {
            int count = 0;
            while (Interlocked.CompareExchange(ref owner, 0, -1) != -1)
            {
                //Lock is owned by someone else.
                WaitBriefly(count++);
            }
            //It's my lock now!
        }

        /// <summary>
        /// Exits the critical section.  This can only be safely called from the same
        /// thread of execution after a corresponding Enter.
        /// </summary>
        public void Exit()
        {
            //To be safe, technically should check the identity of the exiter.
            //But since this is a very low-level, restricted access class,
            //assume that enter has to succeed before exit is tried.
            owner = -1;
        }

        internal void WaitBriefly(int attempt)
        {
            if (attempt % SleepInterval == SleepInterval - 1)
            {
#if WINDOWS
                Thread.Yield();
#else
                Thread.Sleep(0);
#endif
                //TODO: Thread.Yield on windows?
                //Check multithreaded bookmarks performance conscious
                //and .netspinlock
            }
            else
            {
                Thread.SpinWait(3 << attempt);
            }
        }
    }
}