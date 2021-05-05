using System;
using System.Collections.Generic;

namespace Cyclone
{
    using ForceRegistration = KeyValuePair<RigidBody, IForceGenerator>;

    public class ForceRegistry
    {
        private List<ForceRegistration> registrations;

        public ForceRegistry()
        {
            registrations = new List<ForceRegistration>();
        }

        public void Add(RigidBody body, IForceGenerator fg)
        {
            ForceRegistration pair = new ForceRegistration(body, fg);
            registrations.Add(pair);
        }

        public void Remove(RigidBody body, IForceGenerator fg)
        {
            registrations.Remove(new KeyValuePair<RigidBody, IForceGenerator>(body, fg));
        }

        public void Clear()
        {
            registrations.Clear();
        }

        public void UpdateForces(double duration)
        {
            foreach (ForceRegistration registration in registrations)
            {
                registration.Value.UpdateForce(registration.Key, duration);
            }
        }
    }
}
