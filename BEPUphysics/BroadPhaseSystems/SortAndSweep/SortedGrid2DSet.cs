using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using BEPUphysics.ResourceManagement;

namespace BEPUphysics.BroadPhaseSystems.SortAndSweep
{
    internal class SortedGrid2DSet
    {
        internal RawList<GridCell2D> cells = new RawList<GridCell2D>();
        UnsafeResourcePool<GridCell2D> cellPool = new UnsafeResourcePool<GridCell2D>();

        internal int count;


        internal void Add(ref Int2 index, Grid2DEntry entry)
        {

            //Take an index.  See if it's taken in the set.
            //If it's already there, then add the entry to the cell.
            //If it's not already there, create a new cell and add the entry to the cell and insert it at the index located.

            int sortingHash = index.GetSortingHash();
            int minIndex = 0; //inclusive
            int maxIndex = count; //exclusive
            int i = 0;
            while (maxIndex - minIndex > 0) //If the testing interval has a length of zero, we've done as much as we can.
            {
                i = (maxIndex + minIndex) / 2;
                if (cells.Elements[i].sortingHash > sortingHash)
                    maxIndex = i;
                else if (cells.Elements[i].sortingHash < sortingHash)
                    minIndex = ++i;
                else
                {
                    //Found an equal sorting hash!
                    //The hash can collide, and we cannot add an entry to 
                    //an incorrect index.  It would break the 'cell responsibility' 
                    //used by the cell update process to avoid duplicate overlaps.
                    //So, check if the index we found is ACTUALLY correct.
                    if (cells.Elements[i].cellIndex.Y == index.Y && cells.Elements[i].cellIndex.Z == index.Z)
                    {
                        cells.Elements[i].Add(entry);
                        return;
                    }
                    //If it was not the correct index, let it continue searching.
                }

            }
            var cell = cellPool.Take();
            cell.Initialize(ref index, sortingHash);
            cell.Add(entry);
            cells.Insert(i, cell);
            count++;
            
        }

        internal void Remove(ref Int2 index, Grid2DEntry entry)
        {
            int sortingHash = index.GetSortingHash();
            int minIndex = 0; //inclusive
            int maxIndex = count; //exclusive
            int i = 0;
            while (maxIndex - minIndex > 0) //If the testing interval has a length of zero, we've done as much as we can.
            {
                i = (maxIndex + minIndex) / 2;
                if (cells.Elements[i].sortingHash > sortingHash)
                    maxIndex = i;
                else if (cells.Elements[i].sortingHash < sortingHash)
                    minIndex = ++i;
                else
                {
                    //Found an equal sorting hash!
                    //The hash can collide, and we cannot add an entry to 
                    //an incorrect index.  It would break the 'cell responsibility' 
                    //used by the cell update process to avoid duplicate overlaps.
                    //So, check if the index we found is ACTUALLY correct.
                    if (cells.Elements[i].cellIndex.Y == index.Y && cells.Elements[i].cellIndex.Z == index.Z)
                    {
                        cells.Elements[i].Remove(entry);
                        if (cells.Elements[i].entries.count == 0)
                        {
                            //The cell is now empty.  Give it back to the pool.
                            var toRemove = cells.Elements[i];
                            //There's no cleanup to do on the grid cell.
                            //Its list is empty, and the rest is just value types.
                            cells.RemoveAt(i);
                            cellPool.GiveBack(toRemove);
                            count--;
                        }
                        return;
                    }
                    //If it was not the correct index, let it continue searching.
                }

            }
            //Getting here should be impossible.
            
        }

    }
}
