using BEPUphysics.BroadPhaseEntries.Events;
using BEPUphysics.CollisionShapes;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionRuleManagement;
using System;
using BEPUutilities.DataStructures;

namespace BEPUphysics.BroadPhaseEntries
{
    ///<summary>
    /// Superclass of objects living in the collision detection pipeline
    /// that can result in contacts.
    ///</summary>
    public abstract class Collidable : BroadPhaseEntry
    {
        protected Collidable()
        {
            shapeChangedDelegate = OnShapeChanged;
        }



        protected internal CollisionShape shape; //Having this non-private allows for some very special-casey stuff; see TriangleShape initialization.
        ///<summary>
        /// Gets the shape used by the collidable.
        ///</summary>
        public CollisionShape Shape
        {
            get
            {
                return shape;
            }
            protected set
            {
                if (shape != null && shapeChangedHooked)
                    shape.ShapeChanged -= shapeChangedDelegate;
                shape = value;
                if (shape != null && shapeChangedHooked)
                    shape.ShapeChanged += shapeChangedDelegate;
                OnShapeChanged(shape);
            }
        }

        bool shapeChangedHooked = true;
        /// <summary>
        /// Gets or sets whether the shape changed event is hooked. Setting this modifies the event delegate list on the associated shape, if any shape exists.
        /// If no shape exists, getting this property returns whether the event would be hooked if a shape was present.
        /// </summary>
        /// <remarks>Yes, this is a hack.</remarks>
        public bool ShapeChangedHooked
        {
            get
            {
                return shapeChangedHooked;
            }
            set
            {
                if (shape != null)
                {
                    if (shapeChangedHooked && !value)
                    {
                        shape.ShapeChanged -= shapeChangedDelegate;
                    }
                    else if (!shapeChangedHooked && value)
                    {
                        shape.ShapeChanged += shapeChangedDelegate;
                    }
                }
                shapeChangedHooked = value;
            }
        }

        protected internal abstract IContactEventTriggerer EventTriggerer { get; }



        /// <summary>
        /// Gets or sets whether or not to ignore shape changes.  When true, changing the collision shape will not force the collidable to perform any updates.
        /// Does not modify the shape changed event delegate list.
        /// </summary>
        public bool IgnoreShapeChanges { get; set; }

        Action<CollisionShape> shapeChangedDelegate;
        protected virtual void OnShapeChanged(CollisionShape collisionShape)
        {
        }


        internal RawList<CollidablePairHandler> pairs = new RawList<CollidablePairHandler>();
        ///<summary>
        /// Gets the list of pairs associated with the collidable.
        /// These pairs are found by the broad phase and are managed by the narrow phase;
        /// they can contain other collidables, entities, and contacts.
        ///</summary>
        public ReadOnlyList<CollidablePairHandler> Pairs
        {
            get
            {
                return new ReadOnlyList<CollidablePairHandler>(pairs);
            }
        }

        ///<summary>
        /// Gets a list of all other collidables that this collidable overlaps.
        ///</summary>
        public CollidableCollection OverlappedCollidables
        {
            get
            {
                return new CollidableCollection(this);
            }
        }

        protected override void CollisionRulesUpdated()
        {
            for (int i = 0; i < pairs.Count; i++)
            {
                pairs[i].CollisionRule = CollisionRules.CollisionRuleCalculator(pairs[i].BroadPhaseOverlap.entryA, pairs[i].BroadPhaseOverlap.entryB);
            }
        }



        internal void AddPair(CollidablePairHandler pair, ref int index)
        {
            index = pairs.Count;
            pairs.Add(pair);
        }

        internal void RemovePair(CollidablePairHandler pair, ref int index)
        {
            if (pairs.Count > index)
            {
                pairs.FastRemoveAt(index);
                if (pairs.Count > index)
                {
                    var endPair = pairs.Elements[index];
                    if (endPair.CollidableA == this)
                        endPair.listIndexA = index;
                    else
                        endPair.listIndexB = index;
                }
            }
            index = -1;
        }


    }


}
