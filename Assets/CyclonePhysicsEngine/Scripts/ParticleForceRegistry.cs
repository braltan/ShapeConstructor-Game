using System;
using System.Collections.Generic;

namespace Cyclone
{
    using ParticleForceRegistration = KeyValuePair<Particle, IParticleForceGenerator>;

    public class ParticleForceRegistry
    {
        private List<ParticleForceRegistration> registrations;

        public ParticleForceRegistry()
        {
            registrations = new List<ParticleForceRegistration>();
        }

        public void Add(Particle particle, IParticleForceGenerator fg)
        {
            ParticleForceRegistration pair = new ParticleForceRegistration(particle, fg);
            registrations.Add(pair);
        }

        public void Remove(Particle particle, IParticleForceGenerator fg)
        {
            registrations.Remove(new KeyValuePair<Particle, IParticleForceGenerator>(particle, fg));
        }

        public void Clear()
        {
            registrations.Clear();
        }

        public void UpdateForces(double duration)
        {
            foreach (ParticleForceRegistration registration in registrations)
            {
                registration.Value.UpdateForce(registration.Key, duration);
            }
        }
    }
}
