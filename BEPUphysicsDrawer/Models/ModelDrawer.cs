using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUphysics.Collidables;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.UpdateableSystems;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Manages and draws models.
    /// </summary>
    public abstract class ModelDrawer
    {
        private readonly Dictionary<object, ModelDisplayObjectBase> displayObjects = new Dictionary<object, ModelDisplayObjectBase>();
        private readonly RasterizerState fillState;

        private readonly List<SelfDrawingModelDisplayObject> selfDrawingDisplayObjects = new List<SelfDrawingModelDisplayObject>();
        private readonly RasterizerState wireframeState;
        protected Texture2D[] textures = new Texture2D[8];



        private static readonly Dictionary<Type, Type> displayTypes = new Dictionary<Type, Type>();
        private static readonly Dictionary<Type, ShapeMeshGetter> shapeMeshGetters = new Dictionary<Type, ShapeMeshGetter>();

        /// <summary>
        /// Gets the map from object types to display object types.
        /// </summary>
        public static Dictionary<Type, Type> DisplayTypes
        {
            get { return displayTypes; }
        }

        /// <summary>
        /// Gets the map from shape object types to methods which can be used to construct the data.
        /// </summary>
        public static Dictionary<Type, ShapeMeshGetter> ShapeMeshGetters
        {
            get { return shapeMeshGetters; }
        }

        static ModelDrawer()
        {
            //Display types are sometimes requested from contexts lacking a convenient reference to a ModelDrawer instance.
            //Having them static simplifies things.
            displayTypes.Add(typeof(FluidVolume), typeof(DisplayFluid));
            displayTypes.Add(typeof(Terrain), typeof(DisplayTerrain));
            displayTypes.Add(typeof(TriangleMesh), typeof(DisplayTriangleMesh));
            displayTypes.Add(typeof(StaticMesh), typeof(DisplayStaticMesh));
            displayTypes.Add(typeof(InstancedMesh), typeof(DisplayInstancedMesh));


            //Entity types are handled through a special case that uses an Entity's Shape to look up one of the ShapeMeshGetters.
            shapeMeshGetters.Add(typeof(ConvexCollidable<BoxShape>), DisplayBox.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<SphereShape>), DisplaySphere.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<CapsuleShape>), DisplayCapsule.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<CylinderShape>), DisplayCylinder.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<ConeShape>), DisplayCone.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<TriangleShape>), DisplayTriangle.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<ConvexHullShape>), DisplayConvexHull.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<MinkowskiSumShape>), DisplayMinkowskiSum.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<WrappedShape>), DisplayWrappedBody.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(ConvexCollidable<TransformableShape>), DisplayTransformable.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(CompoundCollidable), DisplayCompoundBody.GetShapeMeshData);
            shapeMeshGetters.Add(typeof(MobileMeshCollidable), DisplayMobileMesh.GetShapeMeshData);

        }

        protected ModelDrawer(Game game)
        {
            Game = game;
            Color[][] colours = new Color[][] 
            { 
                new Color[1]{new Color(255, 216, 0)},
                new Color[1]{new Color(79, 200, 255)},
                new Color[1]{new Color(255, 0, 0)},
                new Color[1]{new Color(177, 0, 254)},
                new Color[1]{new Color(255, 130, 151)},
                new Color[1]{new Color(254, 106, 0)},
                new Color[1]{new Color(168, 165, 255)},
                new Color[1]{new Color(0, 254, 33)}
            };

            for (int i = 0; i < 8; i++)
            {
                textures[i] = new Texture2D(Game.GraphicsDevice, 1, 1);
                textures[i].SetData<Color>(colours[i]);
            }


            fillState = new RasterizerState();
            wireframeState = new RasterizerState();
            wireframeState.FillMode = FillMode.WireFrame;
        }



        /// <summary>
        /// Gets the game using this ModelDrawer.
        /// </summary>
        public Game Game { get; private set; }

        /// <summary>
        /// Gets or sets whether or not the model drawer is drawing wireframes.
        /// </summary>
        public bool IsWireframe { get; set; }

        /// <summary>
        /// Constructs a new display object for an object.
        /// </summary>
        /// <param name="objectToDisplay">Object to create a display object for.</param>
        /// <returns>Display object for an object.</returns>
        internal ModelDisplayObjectBase GetDisplayObject(object objectToDisplay)
        {
            Type displayType;
            if (!displayObjects.ContainsKey(objectToDisplay))
            {
                Entity e;
                if (displayTypes.TryGetValue(objectToDisplay.GetType(), out displayType))
                {
#if !WINDOWS
                    return (ModelDisplayObjectBase)displayType.GetConstructor(
                                                     new Type[] { typeof(ModelDrawer), objectToDisplay.GetType() })
                                                     .Invoke(new object[] { this, objectToDisplay });
#else
                    return (ModelDisplayObjectBase)Activator.CreateInstance(displayType, new[] { this, objectToDisplay });
#endif
                }
                else if ((e = objectToDisplay as Entity) != null)
                {
                    return new DisplayEntity(this, e);
                }

            }
            return null;
        }


        /// <summary>
        /// Attempts to add an object to the ModelDrawer.
        /// </summary>
        /// <param name="objectToDisplay">Object to be added to the model drawer.</param>
        /// <returns>ModelDisplayObject created for the object.  Null if it couldn't be added.</returns>
        public ModelDisplayObjectBase Add(object objectToDisplay)
        {
            ModelDisplayObjectBase displayObject = GetDisplayObject(objectToDisplay);
            if (displayObject != null)
            {
                Add(displayObject);
                displayObjects.Add(objectToDisplay, displayObject);
                return displayObject;
            }
            return null; //Couldn't add it.
        }

        /// <summary>
        /// Adds the display object to the drawer.
        /// </summary>
        /// <param name="displayObject">Display object to add.</param>
        /// <returns>Whether or not the display object was added.</returns>
        public bool Add(SelfDrawingModelDisplayObject displayObject)
        {
            if (!selfDrawingDisplayObjects.Contains(displayObject))
            {
                selfDrawingDisplayObjects.Add(displayObject);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Adds a display object to the drawer.
        /// </summary>
        /// <param name="displayObject">Display object to add.</param>
        protected abstract void Add(ModelDisplayObjectBase displayObject);

        /// <summary>
        /// Removes an object from the drawer.
        /// </summary>
        /// <param name="objectToRemove">Object to remove.</param>
        public void Remove(object objectToRemove)
        {
            Remove(displayObjects[objectToRemove]);
            displayObjects.Remove(objectToRemove);
        }

        /// <summary>
        /// Removes an object from the drawer.
        /// </summary>
        /// <param name="displayObject">Display object to remove.</param>
        public void Remove(SelfDrawingModelDisplayObject displayObject)
        {
            selfDrawingDisplayObjects.Remove(displayObject);
        }


        /// <summary>
        /// Removes a display object from the drawer.
        /// </summary>
        /// <param name="displayObject">Object to remove.</param>
        protected abstract void Remove(ModelDisplayObjectBase displayObject);

        /// <summary>
        /// Cleans out the model drawer of any existing display objects.
        /// </summary>
        public void Clear()
        {
            displayObjects.Clear();
            selfDrawingDisplayObjects.Clear();
            ClearManagedModels();
        }

        /// <summary>
        /// Cleans out any data contained by derived drawers.
        /// </summary>
        protected abstract void ClearManagedModels();

        /// <summary>
        /// Determines if the object has an associated display object in this drawer.
        /// </summary>
        /// <param name="displayedObject">Object to check for in the drawer.</param>
        /// <returns>Whether or not the object has an associated display object in this drawer.</returns>
        public bool Contains(object displayedObject)
        {
            return displayObjects.ContainsKey(displayedObject);
        }

        /// <summary>
        /// Updates the drawer and its components.
        /// </summary>
        public void Update()
        {
            foreach (SelfDrawingModelDisplayObject displayObject in selfDrawingDisplayObjects)
                displayObject.Update();
            UpdateManagedModels();
        }

        /// <summary>
        /// Updates the drawer's technique.
        /// </summary>
        protected abstract void UpdateManagedModels();

        /// <summary>
        /// Draws the drawer's models.
        /// </summary>
        /// <param name="viewMatrix">View matrix to use to draw the objects.</param>
        /// <param name="projectionMatrix">Projection matrix to use to draw the objects.</param>
        public void Draw(Matrix viewMatrix, Matrix projectionMatrix)
        {
            Game.GraphicsDevice.RasterizerState = IsWireframe ? wireframeState : fillState;

            Game.GraphicsDevice.BlendState = BlendState.Opaque;
            Game.GraphicsDevice.DepthStencilState = DepthStencilState.Default;

            foreach (SelfDrawingModelDisplayObject displayObject in selfDrawingDisplayObjects)
                displayObject.Draw(viewMatrix, projectionMatrix);
            DrawManagedModels(viewMatrix, projectionMatrix);
        }

        /// <summary>
        /// Draws the models managed by the drawer using the appropriate technique.
        /// </summary>
        /// <param name="viewMatrix">View matrix to use to draw the objects.</param>
        /// <param name="projectionMatrix">Projection matrix to use to draw the objects.</param>
        protected abstract void DrawManagedModels(Matrix viewMatrix, Matrix projectionMatrix);

        public delegate void ShapeMeshGetter(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices);
    }
}