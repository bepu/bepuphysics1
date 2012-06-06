
using System;
using System.Diagnostics;
using BEPUphysics;
using System.Threading;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Superclass of the sample simulations.
    /// </summary>
    public abstract class Demo
    {
        private int accumulatedPhysicsFrames;
        private double accumulatedPhysicsTime;
        private double previousTimeMeasurement;

        protected Demo(DemosGame game)
        {
            Game = game;
            Space = new Space();
            //This section lets the engine know that it can make use of multithreaded systems
            //by adding threads to its thread pool.
#if XBOX360
            Space.ThreadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 1 }); }, null);
            Space.ThreadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 3 }); }, null);
            Space.ThreadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 4 }); }, null);
            Space.ThreadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 5 }); }, null);

#else
            if (Environment.ProcessorCount > 1)
            {
                for (int i = 0; i < Environment.ProcessorCount; i++)
                {
                    Space.ThreadManager.AddThread();
                }
            }
#endif
            game.Camera.Yaw = 0;
            game.Camera.Pitch = 0;
        }

        /// <summary>
        /// Gets the average time spent per frame in the physics simulation.
        /// </summary>
        public double PhysicsTime { get; private set; }

        /// <summary>
        /// Gets the name of the demo.
        /// </summary>
        public abstract string Name { get; }

        /// <summary>
        /// Gets the game this demo belongs to.
        /// </summary>
        public DemosGame Game { get; private set; }

        /// <summary>
        /// Gets or sets the space this simulation runs in.
        /// </summary>
        public Space Space { get; set; }

        /// <summary>
        /// Updates the game.
        /// </summary>
        /// <param name="dt">Game timestep.</param>
        public virtual void Update(float dt)
        {
            long startTime = Stopwatch.GetTimestamp();

            //Update the simulation.
            //Pass in dt to the function to use internal timestepping, if desired.
            //Using internal time stepping usually works best when the interpolation is also used.
            //Check out the asynchronous updating documentation for an example (though you don't have to use a separate thread to use interpolation).
            Space.Update();


            long endTime = Stopwatch.GetTimestamp();
            accumulatedPhysicsTime += (endTime - startTime) / (double)Stopwatch.Frequency;
            accumulatedPhysicsFrames++;
            previousTimeMeasurement += dt;
            if (previousTimeMeasurement > .3f)
            {
                previousTimeMeasurement -= .3f;
                PhysicsTime = accumulatedPhysicsTime / accumulatedPhysicsFrames;
                accumulatedPhysicsTime = 0;
                accumulatedPhysicsFrames = 0;
            }

        }

        /// <summary>
        /// Draws any extra UI components associated with this demo.
        /// </summary>
        public virtual void DrawUI()
        {
        }

        /// <summary>
        /// Performs arbitrary drawing associated with this demo.
        /// </summary>
        public virtual void Draw()
        {

        }

        /// <summary>
        /// Cleans up this simulation before moving to the next one.
        /// </summary>
        public virtual void CleanUp()
        {
            //Undo any in-demo configuration.
            ConfigurationHelper.ApplyDefaultSettings(Space);
            Space.Dispose();
        }
    }
}