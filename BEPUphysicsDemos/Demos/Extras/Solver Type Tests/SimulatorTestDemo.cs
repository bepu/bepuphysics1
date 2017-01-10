using System;
using System.Collections.Generic;
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
            int width = 30;
            int height = 10;
            int length = 30;
            float spacing = 3;
            var dynamics = new LinearDynamic[width, height, length];
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

            BuildSimulation(new Vector3(-65, 0, 0), jacobiSimulator);
            BuildSimulation(new Vector3(65, 0, 0), sequentialImpulsesSimulator);

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

            //jacobiSimulator.Gravity = new Vector3();
            //sequentialImpulsesSimulator.Gravity = new Vector3();

            game.Camera.Position = new Vector3(0, 36, 200);
        }

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
                        color = Color.Lerp(new Color(1f, 1f, 1f), new Color(1f, 0f, 0f), (threshold - distance) / threshold);
                        lines[index] = new VertexPositionColor(MathConverter.Convert(planeConstraint.Dynamic.Position), color);
                        lines[index + 1] = new VertexPositionColor(MathConverter.Convert(planeConstraint.Dynamic.Position - planeConstraint.Plane.Normal * distance), color);
                    }
                    else
                    {
                        //Negative! Penetrating.
                        var penetratingColor = new Color(1f, 0f, 0f);
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


        private Random random = new Random(10);
        private void ShakeDynamics<T>(T dynamics) where T : IList<LinearDynamic>
        {
            for (int i = 0; i < dynamics.Count; ++i)
            {
                dynamics[i].Velocity += 5 * new Vector3(2 * (float)random.NextDouble() - 1, (float)random.NextDouble(), 2 * (float)random.NextDouble() - 1);
            }
        }


        public override void Update(float dt)
        {
            if (Game.WasKeyPressed(Microsoft.Xna.Framework.Input.Keys.P))
            {
                ShakeDynamics(jacobiSimulator.Dynamics);
                ShakeDynamics(sequentialImpulsesSimulator.Dynamics);
            }

            jacobiSimulator.Update(1 / 60f, Space.ParallelLooper);
            sequentialImpulsesSimulator.Update(1 / 60f, Space.ParallelLooper);

            //Update the dynamics visualizers.
            if (Game.displayEntities)
            {
                foreach (var visualizer in jacobiDynamicVisualizers)
                {
                    visualizer.UpdateDisplay();
                }
                foreach (var visualizer in sequentialImpulsesDynamicVisualizers)
                {
                    visualizer.UpdateDisplay();
                }
            }

            if (Game.displayConstraints)
            {
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
            }

            base.Update(dt);
        }


        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("Jacobi total time (ms): ", 1000 * jacobiSimulator.TotalTime, 2, new Microsoft.Xna.Framework.Vector2(10f, 20f));
            Game.DataTextDrawer.Draw("Jacobi solve time (ms): ", 1000 * jacobiSimulator.SolveTime, 2, new Microsoft.Xna.Framework.Vector2(10f, 40f));
            Game.DataTextDrawer.Draw("Sequential total time (ms): ", 1000 * sequentialImpulsesSimulator.TotalTime, 2, new Microsoft.Xna.Framework.Vector2(10, 70));
            Game.DataTextDrawer.Draw("Sequential solve time (ms): ", 1000 * sequentialImpulsesSimulator.SolveTime, 2, new Microsoft.Xna.Framework.Vector2(10, 90));
            base.DrawUI();
        }

        public override void Draw()
        {
            if (Game.displayConstraints)
            {
                var effect = Game.LineDrawer;
                foreach (var pass in effect.CurrentTechnique.Passes)
                {
                    pass.Apply();
                    Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, jacobiConstraintLines, 0, jacobiConstraintLines.Length / 2);
                    Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, sequentialImpulsesConstraintLines, 0, sequentialImpulsesConstraintLines.Length / 2);
                }
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