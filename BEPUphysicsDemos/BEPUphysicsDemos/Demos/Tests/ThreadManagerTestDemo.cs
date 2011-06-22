//using BEPUphysics.Collidables;
//using BEPUphysics.Entities;
//using BEPUphysics.Entities.Prefabs;
//using BEPUphysics.PositionUpdating;
//using Microsoft.Xna.Framework;
//using Microsoft.Xna.Framework.Input;
//using System.Diagnostics;
//using System.Collections.Generic;
//using System;
//using BEPUphysics.MathExtensions;
//using BEPUphysics.CollisionShapes.ConvexShapes;
//using BEPUphysics.DataStructures;
//using Microsoft.Xna.Framework.Graphics;
//using BEPUphysics.Collidables.MobileCollidables;
//using BEPUphysics.CollisionShapes;
//using BEPUphysics.Settings;
//using BEPUphysics.NarrowPhaseSystems.Pairs;
//using BEPUphysics.CollisionRuleManagement;
//using BEPUphysics.BroadPhaseSystems;
//using BEPUphysics.Constraints;
//using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
//using BEPUphysics.Constraints.SolverGroups;
//using BEPUphysics.CollisionTests.CollisionAlgorithms;
//using BEPUphysics.CollisionTests;
//using BEPUphysics;
//using BEPUphysics.EntityStateManagement;
//using BEPUphysics.ResourceManagement;
//using BEPUphysics.Materials;
//using System.Threading;
//using BEPUphysics.Threading;

//namespace BEPUphysicsDemos.Demos.Tests
//{
//    /// <summary>
//    /// Demo showing a wall of blocks stacked up.
//    /// </summary>
//    public class TestDemo4 : StandardDemo
//    {

//        /// <summary>
//        /// Constructs a new demo.
//        /// </summary>
//        /// <param name="game">Game owning this demo.</param>
//        public TestDemo4(DemosGame game)
//            : base(game)
//        {
//            for (int i = 0; i < 1; i++)
//            {
//                lockedList.Add(i);
//            }

//            int numElements = 1000;
//            for (int i = 0; i < numElements; i++)
//            {
//                list.Add(i);
//            }

       

//            int numRuns = 1000;
//            loopBodyNoLock = LoopBodyNoLock;
//            loopBodySpinLock = LoopBodySpinLock;
//            loopBodySpinLockBarrier = LoopBodySpinLockBarrier;
//            loopBodyBepuSpinLock = LoopBodyBepuSpinLock;
//            loopBodyLock = LoopBodyLock;

//            //SINGLE THREADED
//            double startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            for (int i = 0; i < numRuns; i++)
//            {
//                for (int j = 0; j < numElements; j++)
//                {
//                    LoopBodyNoLock(j);
//                }
//            }
//            double endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            singleThreadedTime = (endTime - startTime) / numRuns;


//            #region Normal Lock
//            //SPECIALIZED
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            InitializeManager(new SpecializedThreadManager());
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodyLock);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            specializedTimeLock = (endTime - startTime) / numRuns;

//            ////THREAD TASK MANAGER
//            //InitializeManager(new ThreadTaskManager());
//            //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //for (int i = 0; i < numRuns; i++)
//            //{
//            //    Space.ThreadManager.ForLoop(0, numElements, loopBodyLock);
//            //}
//            //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //threadTaskTimeLock = (endTime - startTime) / numRuns;

//            //TPL
//            InitializeManager(new ThreadManagerTPL());
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodyLock);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            TPLtimeLock = (endTime - startTime) / numRuns;
//            #endregion

//            #region Spin Lock no barrier
//            //SPECIALIZED
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            InitializeManager(new SpecializedThreadManager());
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodySpinLock);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            specializedTimeSpinLock = (endTime - startTime) / numRuns;

//            ////THREAD TASK MANAGER
//            //InitializeManager(new ThreadTaskManager());
//            //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //for (int i = 0; i < numRuns; i++)
//            //{
//            //    Space.ThreadManager.ForLoop(0, numElements, loopBodySpinLock);
//            //}
//            //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //threadTaskTimeSpinLock = (endTime - startTime) / numRuns;

//            //TPL
//            InitializeManager(new ThreadManagerTPL());
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodySpinLock);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            TPLtimeSpinLock = (endTime - startTime) / numRuns;
//            #endregion


//            #region Spin Lock with barrier
//            //SPECIALIZED
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            InitializeManager(new SpecializedThreadManager());
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodySpinLockBarrier);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            specializedTimeSpinLockBarrier = (endTime - startTime) / numRuns;

//            ////THREAD TASK MANAGER
//            //InitializeManager(new ThreadTaskManager());
//            //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //for (int i = 0; i < numRuns; i++)
//            //{
//            //    Space.ThreadManager.ForLoop(0, numElements, loopBodySpinLockBarrier);
//            //}
//            //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //threadTaskTimeSpinLockBarrier = (endTime - startTime) / numRuns;

//            //TPL
//            InitializeManager(new ThreadManagerTPL());
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodySpinLockBarrier);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            TPLtimeSpinLockBarrier = (endTime - startTime) / numRuns;
//            #endregion

//            #region Bepu Spin Lock
//            //SPECIALIZED
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            InitializeManager(new SpecializedThreadManager());
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodyBepuSpinLock);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            specializedTimeBepuLock = (endTime - startTime) / numRuns;

//            ////THREAD TASK MANAGER
//            //InitializeManager(new ThreadTaskManager());
//            //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //for (int i = 0; i < numRuns; i++)
//            //{
//            //    Space.ThreadManager.ForLoop(0, numElements, loopBodyBepuSpinLock);
//            //}
//            //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            //threadTaskTimeBepuLock = (endTime - startTime) / numRuns;

//            //TPL
//            InitializeManager(new ThreadManagerTPL());
//            startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            for (int i = 0; i < numRuns; i++)
//            {
//                Space.ThreadManager.ForLoop(0, numElements, loopBodyBepuSpinLock);
//            }
//            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
//            TPLtimeBepuLock = (endTime - startTime) / numRuns;
//            #endregion


//        }

//        void InitializeManager(IThreadManager threadManager)
//        {
//            Space.ThreadManager.Dispose();
//            Space.ThreadManager = threadManager;
//            if (Environment.ProcessorCount > 1)
//            {
//                for (int i = 0; i < Environment.ProcessorCount; i++)
//                {
//                    Space.ThreadManager.AddThread();
//                }
//            }
//        }

//        RawList<double> list = new RawList<double>();

//        Action<int> loopBodyLock;
//        void LoopBodyLock(int i)
//        {
//            Work(i);
//            lock (list)
//            {
//                DoSharedWork(i);
//            }
//        }

//        Action<int> loopBodySpinLock;
//        void LoopBodySpinLock(int i)
//        {
//            Work(i);
//            bool lockTaken = false;
//            locker.Enter(ref lockTaken);
//            DoSharedWork(i);
//            locker.Exit(false);
//        }

//        Action<int> loopBodySpinLockBarrier;
//        void LoopBodySpinLockBarrier(int i)
//        {
//            Work(i);
//            bool lockTaken = false;
//            locker.Enter(ref lockTaken);
//            DoSharedWork(i);
//            locker.Exit(true);
//        }

//        Action<int> loopBodyBepuSpinLock;
//        void LoopBodyBepuSpinLock(int i)
//        {
//            Work(i);
//            bepuLocker.Enter();
//            DoSharedWork(i);
//            bepuLocker.Exit();
//        }

//        Action<int> loopBodyNoLock;
//        void LoopBodyNoLock(int i)
//        {
//            Work(i);
//            DoSharedWork(i);
//        }

//        void DoSharedWork(int i)
//        {
//            if (lockedList.Contains(list.Elements[i]))
//                list.Elements[i]++;
//        }

//        void Work(int i)
//        {
//            for (int j = 0; j <20; j++)
//                list.Elements[i] = Math.Cos(Math.Sin(Math.Sqrt(list.Elements[i] + 8) * 100.5) % Math.Pow(list.Elements[i], .9f));
//        }

//        RawList<double> lockedList = new RawList<double>();

//        BEPUphysics.Threading.SpinLock bepuLocker = new BEPUphysics.Threading.SpinLock();
//        System.Threading.SpinLock locker = new System.Threading.SpinLock();

//        double singleThreadedTime;

//        double specializedTimeLock;
//        double threadTaskTimeLock;
//        double TPLtimeLock;


//        double specializedTimeSpinLock;
//        double threadTaskTimeSpinLock;
//        double TPLtimeSpinLock;


//        double specializedTimeSpinLockBarrier;
//        double threadTaskTimeSpinLockBarrier;
//        double TPLtimeSpinLockBarrier;


//        double specializedTimeBepuLock;
//        double threadTaskTimeBepuLock;
//        double TPLtimeBepuLock;

//        public override void DrawUI()
//        {
//            base.DrawUI();
//            Game.DataTextDrawer.Draw("Time per SingleThreaded:    ", singleThreadedTime * 1e6, new Vector2(50, 50));

//            Game.DataTextDrawer.Draw("Time per Specialized lock:    ", specializedTimeLock * 1e6, new Vector2(50, 80));
//            Game.DataTextDrawer.Draw("Time per ThreadTask lock:    ", threadTaskTimeLock * 1e6, new Vector2(50, 110));
//            Game.DataTextDrawer.Draw("Time per TPL lock:    ", TPLtimeLock * 1e6, new Vector2(50, 140));

//            Game.DataTextDrawer.Draw("Time per Specialized spin no barrier:    ", specializedTimeSpinLock * 1e6, new Vector2(50, 180));
//            Game.DataTextDrawer.Draw("Time per ThreadTask spin no barrier:    ", threadTaskTimeSpinLock * 1e6, new Vector2(50, 210));
//            Game.DataTextDrawer.Draw("Time per TPL spin no barrier:    ", TPLtimeSpinLock * 1e6, new Vector2(50, 240));

//            Game.DataTextDrawer.Draw("Time per Specialized spin barrier:    ", specializedTimeSpinLockBarrier * 1e6, new Vector2(50, 280));
//            Game.DataTextDrawer.Draw("Time per ThreadTask spin barrier:    ", threadTaskTimeSpinLockBarrier * 1e6, new Vector2(50, 310));
//            Game.DataTextDrawer.Draw("Time per TPL spin barrier:    ", TPLtimeSpinLockBarrier * 1e6, new Vector2(50, 340));

//            Game.DataTextDrawer.Draw("Time per Specialized bepu lock:    ", specializedTimeBepuLock * 1e6, new Vector2(50, 380));
//            Game.DataTextDrawer.Draw("Time per ThreadTask bepu lock:    ", threadTaskTimeBepuLock * 1e6, new Vector2(50, 410));
//            Game.DataTextDrawer.Draw("Time per TPL bepu lock:    ", TPLtimeBepuLock * 1e6, new Vector2(50, 440));
//        }



//        /// <summary>
//        /// Gets the name of the simulation.
//        /// </summary>
//        public override string Name
//        {
//            get { return "Test4"; }
//        }

//    }
//}