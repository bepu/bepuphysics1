using System.Collections.Generic;
using System.Collections.ObjectModel;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Base class of LineDisplayObjects.
    /// </summary>
    public abstract class LineDisplayObjectBase
    {
        private readonly ReadOnlyCollection<Line> myLinesReadOnly;
        private bool myIsActive = true;

        protected List<Line> myLines = new List<Line>();

        protected LineDisplayObjectBase(LineDrawer drawer)
        {
            Drawer = drawer;
            myLinesReadOnly = new ReadOnlyCollection<Line>(myLines);
        }

        /// <summary>
        /// Gets the drawer that this display object belongs to.
        /// </summary>
        public LineDrawer Drawer { get; private set; }

        /// <summary>
        /// Gets or sets whether or not this display object is active.
        /// </summary>
        public bool IsDrawing
        {
            get { return myIsActive; }
            set
            {
                if (myIsActive && !value)
                    foreach (Line line in myLines)
                    {
                        Drawer.Deactivate(line);
                    }
                else if (!myIsActive && value)
                    foreach (Line line in myLines)
                    {
                        Drawer.Activate(line);
                    }
                myIsActive = value;
            }
        }

        public abstract bool IsActive { get; }

        /// <summary>
        /// Gets the list of lines used by this display object.
        /// </summary>
        public ReadOnlyCollection<Line> Lines
        {
            get { return myLinesReadOnly; }
        }

        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public abstract void Update();


        /// <summary>
        /// Removes the lines used by the display constraint from its line drawer.
        /// </summary>
        public void RemoveLines()
        {
            foreach (Line line in myLines)
            {
                Drawer.RemoveLine(line);
            }
        }

    }
}