using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;

namespace BEPUphysics.BroadPhaseSystems
{
    public class SweepAndPrune : BroadPhase
    {
        Comparer sorter = new Comparer();
        class Comparer : IComparer<BroadPhaseEntry>
        {
            public int Compare(BroadPhaseEntry x, BroadPhaseEntry y)
            {
                return x.boundingBox.Min.X < y.boundingBox.Min.X ? -1 : x.boundingBox.Min.X > y.boundingBox.Min.X ? 1 : 0;
            }
        }

        RawList<BroadPhaseEntry> entries = new RawList<BroadPhaseEntry>();
        public override void Add(BroadPhaseEntry entry)
        {
            entries.Add(entry);
        }

        public override void Remove(BroadPhaseEntry entry)
        {
            entries.Remove(entry);
        }

        protected override void UpdateMultithreaded()
        {
            UpdateSingleThreaded();
        }

        protected override void UpdateSingleThreaded()
        {
            Overlaps.Clear();
            entries.Sort(sorter);
            for (int i = 0; i < entries.count; i++)
            {
                for (int j = i + 1; j < entries.count && entries.Elements[i].boundingBox.Max.X > entries.Elements[j].boundingBox.Min.X; j++)
                {
                    bool intersects;
                    entries.Elements[i].boundingBox.Intersects(ref entries.Elements[j].boundingBox, out intersects);
                    if (intersects)
                    {
                        TryToAddOverlap(entries.Elements[i], entries.Elements[j]);
                    }
                }
            }
        }
    }
}
