using System;

namespace BasicSetupDemo
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (BasicSetupGame game = new BasicSetupGame())
            {
                game.Run();
            }
        }
    }
}

