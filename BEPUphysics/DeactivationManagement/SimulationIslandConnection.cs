using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;

namespace BEPUphysics.DeactivationManagement
{
    /// <summary>
    /// Connects simulation island members together.
    /// </summary>
    public class SimulationIslandConnection
    {
        internal RawList<SimulationIslandMember> members = new RawList<SimulationIslandMember>(2);
        /// <summary>
        /// Gets a list of members connected by the connection.
        /// </summary>
        public RawList<SimulationIslandMember> Members
        {
            get
            {
                return members;
            }
        }

        /// <summary>
        /// Gets or sets the owner of the connection.
        /// </summary>
        public ISimulationIslandConnectionOwner Owner
        {
            get;
            set;
        }

        /// <summary>
        /// Gets whether or not this connection is going to be removed
        /// by the next DeactivationManager stage run.  Connections
        /// slated for removal should not be considered to be part of
        /// a member's 'real' connections.
        /// </summary>
        public bool SlatedForRemoval { get; internal set; }

        
        /// <summary>
        /// Adds the connection to the connected members.
        /// </summary>
        public void AddReferencesToConnectedMembers()
        {
            //Add back the references to this to entities
            for (int i = 0; i < members.count; i++)
            {
                members.Elements[i].AddConnectionReference(this);
            }
        }

        /// <summary>
        /// Removes the connection from the connected members.
        /// </summary>
        public void RemoveReferencesFromConnectedMembers()
        {
            //Clean out the references entities may have had to this solver updateable.
            for (int i = 0; i < members.count; i++)
            {
                members.Elements[i].RemoveConnectionReference(this);
            }
        }

        /// <summary>
        /// Gets or sets the deactivation manager that owns the connection.
        /// </summary>
        public DeactivationManager DeactivationManager { get; internal set; }


        internal void CleanUp()
        {
            SlatedForRemoval = false;
            members.Clear();
            Owner = null;
            DeactivationManager = null;
        }
    }
}
