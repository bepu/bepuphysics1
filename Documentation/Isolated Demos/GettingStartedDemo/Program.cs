using System;

namespace GettingStartedDemo
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (GettingStartedGame game = new GettingStartedGame())
            {
                game.Run();
            }
        }
    }
}

