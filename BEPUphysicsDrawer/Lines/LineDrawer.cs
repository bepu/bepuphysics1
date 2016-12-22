using System;
using System.Collections.Generic;
using BEPUphysics.Constraints;
using BEPUphysics.Constraints.SingleEntity;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUutilities.DataStructures;
using ConversionHelper;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Manages the graphical representation of joints and constraints.
    /// </summary>
    public class LineDrawer : IDisposable
    {
        private readonly Queue<int> availableIndices = new Queue<int>();
        private readonly BlendState blendState;

        private Dictionary<object, LineDisplayObjectBase> displayMapping = new Dictionary<object, LineDisplayObjectBase>();
        private readonly Game game;
        private readonly BasicEffect lineDrawer;

        /// <summary>
        /// Gets the mapping between added objects and their display representations.
        /// </summary>
        public ReadOnlyDictionary<object, LineDisplayObjectBase> DisplayMapping
        {
            get { return new ReadOnlyDictionary<object, LineDisplayObjectBase>(displayMapping); }
        }


        private readonly Dictionary<Type, Type> displayTypes = new Dictionary<Type, Type>();
        private int firstOpenIndex;

        /// <summary>
        /// Line vertices buffer used by the constraint graphics.
        /// </summary>
        internal VertexPositionColor[] vertices = new VertexPositionColor[16];

        public LineDrawer(Game game)
        {
            this.game = game;
            lineDrawer = new BasicEffect(game.GraphicsDevice);

            //Set up the default type mapping.   
            displayTypes.Add(typeof(PointOnPlaneJoint), typeof(DisplayPointOnPlaneJoint));
            displayTypes.Add(typeof(SwivelHingeAngularJoint), typeof(DisplaySwivelHingeAngularJoint));
            displayTypes.Add(typeof(PointOnLineJoint), typeof(DisplayPointOnLineJoint));
            displayTypes.Add(typeof(BallSocketJoint), typeof(DisplayBallSocketJoint));
            displayTypes.Add(typeof(TwistJoint), typeof(DisplayTwistJoint));
            displayTypes.Add(typeof(TwistMotor), typeof(DisplayTwistMotor));
            displayTypes.Add(typeof(DistanceLimit), typeof(DisplayDistanceLimit));
            displayTypes.Add(typeof(DistanceJoint), typeof(DisplayDistanceJoint));
            displayTypes.Add(typeof(LinearAxisLimit), typeof(DisplayLinearAxisLimit));
            displayTypes.Add(typeof(EllipseSwingLimit), typeof(DisplayEllipseSwingLimit));
            displayTypes.Add(typeof(SingleEntityLinearMotor), typeof(DisplaySingleEntityLinearMotor));
            displayTypes.Add(typeof(RevoluteLimit), typeof(DisplayRevoluteLimit));

            blendState = new BlendState();
            blendState.ColorSourceBlend = Blend.SourceAlpha;
            blendState.AlphaSourceBlend = Blend.SourceAlpha;
            blendState.AlphaDestinationBlend = Blend.InverseSourceAlpha;
            blendState.ColorDestinationBlend = Blend.InverseSourceAlpha;
        }

        /// <summary>
        /// Gets the map from object types to display object types.
        /// </summary>
        public Dictionary<Type, Type> DisplayTypes
        {
            get { return displayTypes; }
        }

        /// <summary>
        /// Finds a place for a new line in the vertex list.
        /// </summary>
        /// <param name="index">Index for the new line.</param>
        public void GetNewLineIndex(out int index)
        {
            if (availableIndices.Count > 0)
            {
                index = availableIndices.Dequeue();
            }
            else
            {
                //If the array is too small to hold a new line, make it bigger.
                if (firstOpenIndex >= vertices.Length)
                {
                    var resizedArray = new VertexPositionColor[vertices.Length * 2];
                    Array.Copy(vertices, resizedArray, vertices.Length);
                    vertices = resizedArray;
                }
                index = firstOpenIndex;
                firstOpenIndex += 2;
            }
        }


        /// <summary>
        /// Makes a line invisible.
        /// </summary>
        /// <param name="line">Line to make invisible.</param>
        public void Deactivate(Line line)
        {
            line.ColorA = new Color(line.ColorA.R, line.ColorA.G, line.ColorA.B, 0);
            line.ColorB = new Color(line.ColorB.R, line.ColorB.G, line.ColorB.B, 0);
        }

        /// <summary>
        /// Makes a line visible.
        /// </summary>
        /// <param name="line">Line to make visible.</param>
        public void Activate(Line line)
        {
            line.ColorA = new Color(line.ColorA.R, line.ColorA.G, line.ColorA.B, 255);
            line.ColorB = new Color(line.ColorB.R, line.ColorB.G, line.ColorB.B, 255);
        }

        /// <summary>
        /// Makes the line's index available to use for other lines.
        /// </summary>
        /// <param name="line">Line to remove.</param>
        public void RemoveLine(Line line)
        {
            availableIndices.Enqueue(line.IndexA);
            //Prevent the 'trash data' from rendering
            vertices[line.IndexA].Color = Color.Transparent;
            vertices[line.IndexB].Color = Color.Transparent;
        }

        /// <summary>
        /// Updates the component.
        /// </summary>
        public void Update()
        {
            //Update vertex positions and data.
            foreach (var displayObject in displayMapping.Values)
            {
                if (displayObject.IsDrawing && displayObject.IsActive)
                    displayObject.Update();
            }
        }

        /// <summary>
        /// Draws the component.
        /// </summary>
        /// <param name="viewMatrix">View matrix to use when rendering the lines.</param>
        /// <param name="projectionMatrix">Projection matrix to use when rendering the lines.</param>
        public void Draw(BEPUutilities.Matrix viewMatrix, BEPUutilities.Matrix projectionMatrix)
        {
            int numElements = firstOpenIndex / 2;

            if (numElements > 0)
            {
                //Set state

                game.GraphicsDevice.BlendState = blendState;

                lineDrawer.LightingEnabled = false;
                lineDrawer.VertexColorEnabled = true;
                lineDrawer.World = Matrix.Identity;
                lineDrawer.View = MathConverter.Convert(viewMatrix);
                lineDrawer.Projection = MathConverter.Convert(projectionMatrix);

                //Draw

                for (int i = 0; i < lineDrawer.CurrentTechnique.Passes.Count; i++)
                {
                    lineDrawer.CurrentTechnique.Passes[i].Apply();

                    game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, vertices, 0, numElements);
                }
            }
        }

        /// <summary>
        /// Adds the object to the rendering system.
        /// </summary>
        /// <param name="o">Object to add.</param>
        /// <returns>Display object created from the object.</returns>
        public LineDisplayObjectBase Add(object o)
        {
            //Create the correct kind of display object for this line drawer.
            Type displayType;
            if (displayTypes.TryGetValue(o.GetType(), out displayType))
            {
#if !WINDOWS
                LineDisplayObjectBase objectAdded = (LineDisplayObjectBase)displayType.GetConstructor(
                                                    new Type[] { o.GetType(), typeof(LineDrawer) })
                                                    .Invoke(new object[] { o, this });
#else
                var objectAdded = (LineDisplayObjectBase)Activator.CreateInstance(displayType, new[] { o, this });
#endif
                displayMapping.Add(o, objectAdded);
                return objectAdded;
            }
            if (o is SolverGroup)
            {
                //Solver groups are special.  If no special type-specific display object 
                //has been registered for a solver group, add every child individually.
                foreach (SolverUpdateable item in (o as SolverGroup).SolverUpdateables)
                {
                    LineDisplayObjectBase objectAdded = Add(item);
                    if (objectAdded != null)
                        objectAdded.IsDrawing = item.IsActiveInSolver; //Match the initial activity of the item.
                }
                return null;
            }
            return null; //Don't have this type registered; don't know what to do with it.
        }

        /// <summary>
        /// Removes the object from the rendering system.
        /// </summary>
        /// <param name="o">Object to remove.</param>
        /// <returns>True if the object was fully removed. False if the object was not found in the drawer, or if it was a solver group that could only be partially removed.</returns>
        public bool Remove(object o)
        {
            LineDisplayObjectBase displayObject;
            if (displayMapping.TryGetValue(o, out displayObject))
            {
                displayObject.RemoveLines();
                displayMapping.Remove(o);
                return true;
            }
            if (o is SolverGroup)
            {
                //Solver groups are special.  If no special type-specific display object 
                //has been registered for a solver group, remove every child individually.
                bool removed = true;
                foreach (var item in (o as SolverGroup).SolverUpdateables)
                {
                    //Only try to remove types which the line drawer could have recognized.
                    Type displayType;
                    if (displayTypes.TryGetValue(item.GetType(), out displayType) && !Remove(item))
                        removed = false;
                }
                return removed;
            }
            return false;
        }


        /// <summary>
        /// Clears out pre-existing data in the line drawer.
        /// </summary>
        public void Clear()
        {
            firstOpenIndex = 0;
            availableIndices.Clear();
            displayMapping.Clear();
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            { 
                disposed = true;
                blendState.Dispose();
                lineDrawer.Dispose();
            }
        }
    }
}