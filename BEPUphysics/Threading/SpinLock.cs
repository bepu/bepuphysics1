using System.Threading;

namespace BEPUphysics.Threading
{
    public class SpinLock
    {
        private const int SleepInterval = 10;
        private int owner = -1;


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
                Thread.Sleep(0);
                //Thread.Yield();
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