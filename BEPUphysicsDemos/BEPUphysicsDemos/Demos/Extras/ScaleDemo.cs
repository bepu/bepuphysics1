using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;

namespace BEPUphysicsDemos.Demos.Extras
{
    /// <summary>
    /// Demonstrates tuning settings for handling different world scales.
    /// </summary>
    public class ScaleDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ScaleDemo(DemosGame game)
            : base(game)
        {
            //Pick a scale!
            //Beware: If you go too far (particularly 0.01 and lower) issues could start to crop up.
            float scale = 1;

            //Load in mesh data and create the collision mesh.
            //The 'mesh' will be a supergiant triangle.
            //Triangles in meshes have a collision detection system which bypasses most numerical issues for collisions on the face of the triangle.
            //Edge collisions fall back to the general case collision detection system which is susceptible to numerical issues at extreme scales.
            //For our simulation, the edges will be too far away to worry about!
            Vector3[] vertices;
            int[] indices;
            vertices = new Vector3[] { new Vector3(-10000, 0, -10000), new Vector3(-10000, 0, 20000), new Vector3(20000, 0, -10000) };
            indices = new int[] { 2, 1, 0 };
            var staticMesh = new StaticMesh(vertices, indices, new AffineTransform(Matrix3x3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, 0, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);

            //Since everything's pretty large, increase the gravity a whole bunch to make things fall fast.
            Space.ForceUpdater.Gravity *= scale;

            //Change the various engine tuning factors so that collision detection and collision response handle the changed scale better.
            ConfigurationHelper.ApplyScale(Space, scale);

            //When dealing with objects that generally have high velocities and accelerations relative to their size, having a shorter time step duration can boost quality
            //a whole lot.  Once the configuration is set properly, most of any remaining 'unsmoothness' in the simulation is due to a lack of temporal resolution; 
            //one discrete step can take an object from a valid state to an unpleasing state due to the high rates of motion.  
            //To simulate the same amount of time with a smaller time step duration requires taking more time steps.
            //This is a quality-performance tradeoff.  If you want to do this, set the time step duration like so:

            //Space.TimeStepSettings.TimeStepDuration = 1 / 120f;

            //And then, in the update, either call the Space.Update() method proportionally more often or use the Space.Update(dt) version, which takes as many timesteps are necessary to simulate dt time.
            //Watch out: when using the internal timestepping method, you may notice slight motion jitter since the number of updates per frame isn't fixed.  Interpolation buffers can be used
            //to address this; check the Asynchronous Update documentation for more information on using internal time stepping.  
            //[Asynchronously updating isn't required to use internal timestepping, but it is a common use case.]

            //Dump some boxes on top of it for fun.
            int numColumns = 8;
            int numRows = 8;
            int numHigh = 1;
            float separation = 2 * scale;
            float baseWidth = 0.5f;
            float baseHeight = 1;
            float baseLength = 1.5f;
            Entity toAdd;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        toAdd = new Box(
                            new Vector3(
                            separation * i - numRows * separation / 2,
                            2 * scale + k * separation,
                            separation * j - numColumns * separation / 2),
                            baseWidth * scale, baseHeight * scale, baseLength * scale, 15);

                        Space.Add(toAdd);
                    }

            //Dump some stuff on top of it that use general case collision detection when they collide with boxes.
            numColumns = 3;
            numRows = 3;
            numHigh = 4;
            separation = 2 * scale;
            baseWidth = 1f;
            baseHeight = 1;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        toAdd = new Cylinder(
                            new Vector3(
                            separation * i - numRows * separation / 2,
                            8 * scale + k * separation,
                            separation * j - numColumns * separation / 2),
                            baseHeight * scale, 0.5f * baseWidth * scale,  15);

                        Space.Add(toAdd);
                    }



            game.Camera.Position = scale * new Vector3(0, 4, 10);
            originalCameraSpeed = freeCameraControlScheme.Speed;
            freeCameraControlScheme.Speed *= scale;


        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Scale Demo"; }
        }

        private float originalCameraSpeed;
        public override void CleanUp()
        {
            freeCameraControlScheme.Speed = originalCameraSpeed;
            base.CleanUp();
        }

    }
}