using System;
namespace BEPUphysicsDemos
{
    internal static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        private static void Main(string[] args)
        {
            using (var game = new DemosGame())
            {
                game.Run();
            }
        }
    }
}