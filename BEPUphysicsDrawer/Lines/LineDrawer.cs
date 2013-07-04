using System;
using System.Collections.Generic;
using BEPUphysics.Constraints;
using BEPUphysics.Constraints.SingleEntity;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Manages the graphical representation of joints and constraints.
    /// </summary>
    public class LineDrawer
    {
        private readonly Queue<int> availableIndices = new Queue<int>();
        private readonly BlendState blendState;

        private readonly List<LineDisplayObjectBase> displayObjects = new List<LineDisplayObjectBase>();
        private readonly Game game;
        private readonly BasicEffect lineDrawer;


        private readonly Dictionary<Type, Type> myDisplayTypes = new Dictionary<Type, Type>();
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
            myDisplayTypes.Add(typeof (PointOnPlaneJoint), typeof (DisplayPointOnPlaneJoint));
            myDisplayTypes.Add(typeof (SwivelHingeAngularJoint), typeof (DisplaySwivelHingeAngularJoint));
            myDisplayTypes.Add(typeof (PointOnLineJoint), typeof (DisplayPointOnLineJoint));
            myDisplayTypes.Add(typeof (BallSocketJoint), typeof (DisplayBallSocketJoint));
            myDisplayTypes.Add(typeof (TwistJoint), typeof (DisplayTwistJoint));
            myDisplayTypes.Add(typeof (DistanceLimit), typeof (DisplayDistanceLimit));
            myDisplayTypes.Add(typeof (DistanceJoint), typeof (DisplayDistanceJoint));
            myDisplayTypes.Add(typeof (LinearAxisLimit), typeof (DisplayLinearAxisLimit));
            myDisplayTypes.Add(typeof (EllipseSwingLimit), typeof (DisplayEllipseSwingLimit));
            myDisplayTypes.Add(typeof (SingleEntityLinearMotor), typeof (DisplaySingleEntityLinearMotor));
            myDisplayTypes.Add(typeof (RevoluteLimit), typeof (DisplayRevoluteLimit));

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
            get { return myDisplayTypes; }
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
            foreach (LineDisplayObjectBase c in displayObjects)
            {
                if (c.IsDrawing && c.IsActive)
                    c.Update();
            }
        }

        /// <summary>
        /// Draws the component.
        /// </summary>
        /// <param name="viewMatrix">View matrix to use when rendering the lines.</param>
        /// <param name="projectionMatrix">Projection matrix to use when rendering the lines.</param>
        public void Draw(Matrix viewMatrix, Matrix projectionMatrix)
        {
            int numElements = firstOpenIndex / 2;

            if (numElements > 0)
            {
                //Set state

                game.GraphicsDevice.BlendState = blendState;

                lineDrawer.LightingEnabled = false;
                lineDrawer.VertexColorEnabled = true;
                lineDrawer.World = Matrix.Identity;
                lineDrawer.View = viewMatrix;
                lineDrawer.Projection = projectionMatrix;

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
            if (myDisplayTypes.TryGetValue(o.GetType(), out displayType))
            {
#if !WINDOWS
                LineDisplayObjectBase objectAdded = (LineDisplayObjectBase)displayType.GetConstructor(
                                                    new Type[] { o.GetType(), typeof(LineDrawer) })
                                                    .Invoke(new object[] { o, this });
#else
                var objectAdded = (LineDisplayObjectBase) Activator.CreateInstance(displayType, new[] {o, this});
#endif
                displayObjects.Add(objectAdded);
                return objectAdded;
            }
            if (o is SolverGroup)
            {
                //Solver groups are special.  If no special type-specific display object 
                //has been registered for a solver group, add every child individually.
                foreach (EntitySolverUpdateable item in (o as SolverGroup).SolverUpdateables)
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
        /// <returns>Display object created from the object that was just removed.  Null if not found.</returns>
        public LineDisplayObjectBase Remove<T>(T o)
        {
            for (int i = 0; i < displayObjects.Count; i++)
            {
                var lineDisplayObject = displayObjects[i] as LineDisplayObject<T>;
                if (lineDisplayObject != null && lineDisplayObject.LineObject.Equals(o))
                {
                    lineDisplayObject.RemoveLines();
                    displayObjects.RemoveAt(i);
                    return lineDisplayObject;
                }
            }
            return null;
        }


        /// <summary>
        /// Clears out pre-existing data in the line drawer.
        /// </summary>
        public void Clear()
        {
            firstOpenIndex = 0;
            availableIndices.Clear();
            displayObjects.Clear();
        }
    }
}