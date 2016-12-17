using System;

namespace MultithreadingDemo
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (MultithreadingGame game = new MultithreadingGame())
            {
                game.Run();
            }
        }
    }
}

