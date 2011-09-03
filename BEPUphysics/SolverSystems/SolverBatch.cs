//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Collections.ObjectModel;
//using BEPUphysics.DataStructures;

//namespace BEPUphysics
//{
//    public class SolverBatch
//    {
//        internal BatchedSolver solver;
//        private List<ISolverBatchUpdateable> solverUpdateables = new List<ISolverBatchUpdateable>();
//        public ReadOnlyCollection<ISolverBatchUpdateable> SolverUpdateables { get; private set; }

//        HashSet<ISimulationIslandMember> containedMembers = new HashSet<ISimulationIslandMember>();

//        public SolverBatch()
//        {
//            SolverUpdateables = new ReadOnlyCollection<ISolverBatchUpdateable>(solverUpdateables);
//        }


//        internal bool TryToAdd(ISolverBatchUpdateable item)
//        {
//            foreach (ISimulationIslandMember e in item.ConnectedMembers)
//            {
//                if (e.IsDynamic && containedMembers.Contains(e))
//                    return false;
//            }
//            foreach (ISimulationIslandMember e in item.ConnectedMembers)
//            {
//                if (e.IsDynamic)
//                    containedMembers.Add(e);
//            }
//            solverUpdateables.Add(item);
//            item.SolverBatch = this;
//            return true;
//        }

//        internal void Remove(ISolverBatchUpdateable item)
//        {
//            if (solverUpdateables.Remove(item))
//            {
//                foreach (ISimulationIslandMember e in item.ConnectedMembers)
//                {
//                    if (e.IsDynamic)
//                        containedMembers.Remove(e);
//                }
//                item.SolverBatch.solver = null;

//                if (solverUpdateables.Count == 0)
//                {
//                    solver.GiveBack(this);
//                }
//            }
//        }


//        internal void Clear()
//        {
//            containedMembers.Clear();
//            solverUpdateables.Clear();
//        }
//    }
//}
