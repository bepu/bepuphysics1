using System.Diagnostics;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysicsDrawer.Models;
using BEPUutilities;
using BEPUutilities.DataStructures;
using ConversionHelper;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Plane = BEPUutilities.Plane;
using Vector3 = BEPUutilities.Vector3;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    /// <summary>
    /// Demo showing a tower of blocks being smashed by a sphere.
    /// </summary>
    public class SimulatorTestDemo : StandardDemo
    {
        private struct DynamicVisualizer
        {
            public readonly DisplayEntityCollidable DisplayCollidable;
            public readonly LinearDynamic Dynamic;

            public DynamicVisualizer(LinearDynamic dynamic, ModelDrawer modelDrawer)
            {
                Dynamic = dynamic;
                DisplayCollidable = new DisplayEntityCollidable(modelDrawer, new ConvexCollidable<BoxShape>(new BoxShape(0.5f, 0.5f, 0.5f)));
                modelDrawer.Add(DisplayCollidable);
            }

            public void UpdateDisplay()
            {
                DisplayCollidable.DisplayedObject.WorldTransform = new RigidTransform(Dynamic.Position);
            }
        }

        private JacobiSimulator jacobiSimulator;
        private SequentialImpulsesSimulator sequentialImpulsesSimulator;

        private DynamicVisualizer[] jacobiDynamicVisualizers;
        private DynamicVisualizer[] sequentialImpulsesDynamicVisualizers;

        private VertexPositionColor[] jacobiConstraintLines;
        private VertexPositionColor[] sequentialImpulsesConstraintLines;


        private void BuildSimulation(Vector3 offset, Simulator simulator)
        {
            //Create a lattice of dynamic objects.
            int width = 5;
            int height = 5;
            int length = 5;
            float spacing = 3;
            var dynamics = new LinearDynamic[5, 5, 5];
            for (int widthIndex = 0; widthIndex < width; ++widthIndex)
            {
                for (int heightIndex = 0; heightIndex < height; ++heightIndex)
                {
                    for (int lengthIndex = 0; lengthIndex < length; ++lengthIndex)
                    {
                        var dynamic = new LinearDynamic(10)
                            {
                                Position = offset + new Vector3(spacing * widthIndex, spacing * heightIndex + 10, spacing * lengthIndex)
                            };
                        dynamics[widthIndex, heightIndex, lengthIndex] = dynamic;
                        simulator.Add(dynamic);
                    }
                }
            }

            Vector3 normal = new Vector3(0, 1, 0);
            Plane plane = new Plane(normal, -Vector3.Dot(offset, normal));

            //Create a bunch of connections between the dynamic objects.
            for (int widthIndex = 0; widthIndex < width; ++widthIndex)
            {
                for (int heightIndex = 0; heightIndex < height; ++heightIndex)
                {
                    for (int lengthIndex = 0; lengthIndex < length; ++lengthIndex)
                    {
                        //Create a connection with the dynamics at +x, +y, +z, +x+y+z, -x+y+z, +x-y+z, +x+y-z
                        //+x
                        if (widthIndex + 1 < width)
                            simulator.Add(new DistanceConstraint(dynamics[widthIndex, heightIndex, lengthIndex], dynamics[widthIndex + 1, heightIndex, lengthIndex]));
                        //+y
                        if (heightIndex + 1 < height)
                            simulator.Add(new DistanceConstraint(dynamics[widthIndex, heightIndex, lengthIndex], dynamics[widthIndex, heightIndex + 1, lengthIndex]));
                        //+z
                        if (lengthIndex + 1 < length)
                            simulator.Add(new DistanceConstraint(dynamics[widthIndex, heightIndex, lengthIndex], dynamics[widthIndex, heightIndex, lengthIndex + 1]));

                        //+x+y+z
                        if (widthIndex + 1 < width && heightIndex + 1 < height && lengthIndex + 1 < length)
                            simulator.Add(new DistanceConstraint(dynamics[widthIndex, heightIndex, lengthIndex], dynamics[widthIndex + 1, heightIndex + 1, lengthIndex + 1]));
                        //-x+y+z
                        if (widthIndex - 1 >= 0 && heightIndex + 1 < height && lengthIndex + 1 < length)
                            simulator.Add(new DistanceConstraint(dynamics[widthIndex, heightIndex, lengthIndex], dynamics[widthIndex - 1, heightIndex + 1, lengthIndex + 1]));
                        //+x-y+z
                        if (widthIndex + 1 < width && heightIndex - 1 >= 0 && lengthIndex + 1 < length)
                            simulator.Add(new DistanceConstraint(dynamics[widthIndex, heightIndex, lengthIndex], dynamics[widthIndex + 1, heightIndex - 1, lengthIndex + 1]));
                        //+x+y-z
                        if (widthIndex + 1 < width && heightIndex + 1 < height && lengthIndex - 1 >= 0)
                            simulator.Add(new DistanceConstraint(dynamics[widthIndex, heightIndex, lengthIndex], dynamics[widthIndex + 1, heightIndex + 1, lengthIndex - 1]));

                        //Create a plane constraint to stop the object from falling.
                        simulator.Add(new PlaneCollisionConstraint(dynamics[widthIndex, heightIndex, lengthIndex], plane));
                    }
                }
            }
        }

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SimulatorTestDemo(DemosGame game)
            : base(game)
        {
            jacobiSimulator = new JacobiSimulator();
            sequentialImpulsesSimulator = new SequentialImpulsesSimulator();

            BuildSimulation(new Vector3(-20, 0, 0), jacobiSimulator);
            BuildSimulation(new Vector3(20, 0, 0), sequentialImpulsesSimulator);

            //Build some visualizers for the simulators.
            var jacobiDynamics = jacobiSimulator.Dynamics;
            jacobiDynamicVisualizers = new DynamicVisualizer[jacobiDynamics.Count];
            var sequentialImpulsesDynamics = sequentialImpulsesSimulator.Dynamics;
            sequentialImpulsesDynamicVisualizers = new DynamicVisualizer[sequentialImpulsesDynamics.Count];

            for (int i = 0; i < jacobiDynamicVisualizers.Length; ++i)
            {
                jacobiDynamicVisualizers[i] = new DynamicVisualizer(jacobiDynamics[i], game.ModelDrawer);
            }
            for (int i = 0; i < sequentialImpulsesDynamicVisualizers.Length; ++i)
            {
                sequentialImpulsesDynamicVisualizers[i] = new DynamicVisualizer(sequentialImpulsesDynamics[i], game.ModelDrawer);
            }

            //Draw some lines for the constraints, too.

            jacobiConstraintLines = new VertexPositionColor[jacobiSimulator.Constraints.Count * 2];
            sequentialImpulsesConstraintLines = new VertexPositionColor[sequentialImpulsesSimulator.Constraints.Count * 2];



            game.Camera.Position = new Vector3(0, 6, 30);
        }

        private double jacobiTime;
        private double sequentialImpulsesTime;

        private void UpdateConstraintLine(int index, VertexPositionColor[] lines, Constraint constraint)
        {
            index <<= 1;
            var color = new Color(0, 0.3f, 0.7f);
            var distanceConstraint = constraint as DistanceConstraint;
            if (distanceConstraint != null)
            {
                lines[index] = new VertexPositionColor(MathConverter.Convert(distanceConstraint.A.Position), color);
                lines[index + 1] = new VertexPositionColor(MathConverter.Convert(distanceConstraint.B.Position), color);
                return;
            }
            var planeConstraint = constraint as PlaneCollisionConstraint;
            if (planeConstraint != null)
            {
                float threshold = 1;
                float distance = planeConstraint.Distance;
                if (distance < threshold)
                {
                    if (distance > 0)
                    {
                        //We're near enough to see it, but not penetrating.
                        Color.Lerp(new Color(1, 1, 1), new Color(1, 0, 0), (threshold - distance) / threshold);
                        lines[index] = new VertexPositionColor(MathConverter.Convert(planeConstraint.Dynamic.Position), color);
                        lines[index + 1] = new VertexPositionColor(MathConverter.Convert(planeConstraint.Dynamic.Position - planeConstraint.Plane.Normal * distance), color);
                    }
                    else
                    {
                        //Negative! Penetrating.
                        var penetratingColor = new Color(1, 0, 0);
                        lines[index] = new VertexPositionColor(MathConverter.Convert(planeConstraint.Dynamic.Position), penetratingColor);
                        lines[index + 1] = new VertexPositionColor(MathConverter.Convert(planeConstraint.Dynamic.Position - planeConstraint.Plane.Normal * distance), penetratingColor);
                    }
                }
                else
                {
                    //Ignore the line.
                    lines[index] = new VertexPositionColor();
                    lines[index + 1] = new VertexPositionColor();
                }
            }
        }

        public override void Update(float dt)
        {
            var start = Stopwatch.GetTimestamp();
            jacobiSimulator.Update(1/60f);
            var end = Stopwatch.GetTimestamp();
            jacobiTime = (1000.0 * (end - start)) / Stopwatch.Frequency;

            start = Stopwatch.GetTimestamp();
            sequentialImpulsesSimulator.Update(1 / 60f);
            end = Stopwatch.GetTimestamp();
            sequentialImpulsesTime = (1000.0 * (end - start)) / Stopwatch.Frequency;

            //Update the dynamics visualizers.
            foreach (var visualizer in jacobiDynamicVisualizers)
            {
                visualizer.UpdateDisplay();
            }
            foreach (var visualizer in sequentialImpulsesDynamicVisualizers)
            {
                visualizer.UpdateDisplay();
            }

            //Update the constraint visualizers.
            var jacobiConstraints = jacobiSimulator.Constraints;
            for (int i = 0; i < jacobiConstraints.Count; ++i)
            {
                UpdateConstraintLine(i, jacobiConstraintLines, jacobiConstraints[i]);
            }
            var sequentialImpulsesConstraints = sequentialImpulsesSimulator.Constraints;
            for (int i = 0; i < sequentialImpulsesConstraints.Count; ++i)
            {
                UpdateConstraintLine(i, sequentialImpulsesConstraintLines, sequentialImpulsesConstraints[i]);
            }

            base.Update(dt);
        }


        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("Jacobi time (ms): ", jacobiTime, 2, new Microsoft.Xna.Framework.Vector2(10f, 20f));
            Game.DataTextDrawer.Draw("Sequential time (ms): ", sequentialImpulsesTime, 2, new Microsoft.Xna.Framework.Vector2(10, 40));
            base.DrawUI();
        }

        public override void Draw()
        {
            var effect = Game.LineDrawer;
            foreach (var pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, jacobiConstraintLines, 0, jacobiConstraintLines.Length / 2);
                Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, sequentialImpulsesConstraintLines, 0, sequentialImpulsesConstraintLines.Length / 2);
            }
            base.Draw();
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Jacobi vs SI Demo"; }
        }
    }
}