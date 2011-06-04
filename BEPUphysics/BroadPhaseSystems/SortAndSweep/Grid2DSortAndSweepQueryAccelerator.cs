using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.BroadPhaseSystems.SortAndSweep
{
    public class Grid2DSortAndSweepQueryAccelerator : IQueryAccelerator
    {
        Grid2DSortAndSweep owner;
        public Grid2DSortAndSweepQueryAccelerator(Grid2DSortAndSweep owner)
        {
            this.owner = owner;
        }

        public bool RayCast(Microsoft.Xna.Framework.Ray ray, IList<BroadPhaseEntry> outputIntersections)
        {
            throw new NotSupportedException("The Grid2DSortAndSweep broad phase cannot accelerate infinite ray casts.  Consider specifying a maximum length or using a broad phase which supports infinite ray casts.");
        }

        public bool RayCast(Microsoft.Xna.Framework.Ray ray, float maximumLength, IList<BroadPhaseEntry> outputIntersections)
        {
            //Use 2d line rasterization.
            //Compute the entry and exit locations in the cell (x value only).
            //Binary search to find the position in the sorted list corresponding to the entry value.
            //Test against each bounding box up until the exit value is reached.
            throw new NotImplementedException();
        }

        public void GetEntries(Microsoft.Xna.Framework.BoundingBox boundingShape, IList<BroadPhaseEntry> overlaps)
        {
            //Select all cells in the bounding box.
            //Binary search to find the position in the sorted list corresponding to the boundingBox.Min.X.
            //Test against each bounding box up until the boundingBox.Max.X is reached.
            throw new NotImplementedException();
        }

        public void GetEntries(Microsoft.Xna.Framework.BoundingSphere boundingShape, IList<BroadPhaseEntry> overlaps)
        {
            //Create a bounding box from the bounding sphere.
            //Binary search to find the position in the sorted list corresponding to the boundingBox.Min.X.
            //Test against each bounding box up until the boundingBox.Max.X is reached.
            //Test the results against the bounding sphere.
            throw new NotImplementedException();
        }

        public void GetEntries(Microsoft.Xna.Framework.BoundingFrustum boundingShape, IList<BroadPhaseEntry> overlaps)
        {
            throw new NotSupportedException("The Grid2DSortAndSweep broad phase cannot accelerate frustum tests.  Consider using a broad phase which supports frustum tests or using a custom solution.");
        }
    }
}
