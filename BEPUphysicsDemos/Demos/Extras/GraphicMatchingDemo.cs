
using BEPUphysics.Entities.Prefabs;
using BEPUphysicsDrawer.Models;
using BEPUutilities;
using ConversionHelper;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos.Extras
{
    /// <summary>
    /// Demo showing how a shape can be constructed from a model and then how to align the graphic with the collision shape.
    /// </summary>
    public class GraphicMatchingDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public GraphicMatchingDemo(DemosGame game)
            : base(game)
        {
            //When creating a collision shape based on a model, the graphic will generally be offset at first.
            //This is because the collision shape computes the center of mass and offsets itself to align with the local origin.
            //That offset must be applied to the graphic to make it match the collision shape.

            //Check out the documentation page for more information: 
            //http://bepuphysics.codeplex.com/wikipage?title=Shape%20Recentering

            var model = game.Content.Load<Model>("guy");
            BEPUutilities.Vector3[] vertices;
            int[] indices;
            ModelDataExtractor.GetVerticesAndIndicesFromModel(model, out vertices, out indices);

            //Create an entity based on the model.
            ConvexHull hull = new ConvexHull(vertices, 10);
            Space.Add(hull);
            //Create a graphic for the hull.  The BEPUphysicsDrawer will also automatically receive a visualization of the convex hull which we can compare our graphic against.
            //The graphic will be offset from the collision shape because we have not attempted to offset it to match the collision shape's origin.
            var graphic = new DisplayEntityModel(hull, model, game.ModelDrawer);
            game.ModelDrawer.Add(graphic);


            //Now let's create another entity from the same vertices.
            hull = new ConvexHull(vertices, 10);
            Space.Add(hull);

            //Note that the prefab entity type ConvexHull uses the ConvexHullShape constructor internally, which outputs the computed center of the shape
            //prior to offset.  The ConvexHull constructor sets the entity's Position to that position so that the initial world space location of the vertices
            //matches what was passed into the constructor.
            //The same process could be performed using this code:

            //Vector3 computedCenter;
            //var shape = new ConvexHullShape(vertices, out computedCenter); //<-- It spits out the computed center here; this is what is used to offset the graphic!
            //var entity = new Entity(shape, 10);
            //entity.Position = computedCenter;
            //Space.Add(entity);

            //For more information about constructing entities, check out the EntityConstructionDemo.

            //But for now, let's just use the prefab entity type.  As mentioned earlier, the constructor set the entity's Position using the computed center.
            //Since we didn't overwrite it with some other position yet, we can still use it.
            graphic = new DisplayEntityModel(hull, model, game.ModelDrawer);
            graphic.LocalTransform = Matrix.CreateTranslation(-hull.Position);
            game.ModelDrawer.Add(graphic);

            //This graphic is perfectly aligned with the collision shape!  Hooray!


            Box ground = new Box(new Vector3(0, -1.5f, 0), 50, 1, 50);
            Space.Add(ground);
            game.Camera.Position = new Vector3(0, 6, 15);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Wall"; }
        }
    }
}