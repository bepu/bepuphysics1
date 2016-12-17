using System;

namespace AsynchronousUpdateDemo
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (AsynchronousUpdateGame game = new AsynchronousUpdateGame())
            {
                game.Run();
            }
        }
    }
}

